#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include <iostream>

#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

// -------------- Declaración de Limites MPC ----------------------------
const float u_max = 2.0;
const float u_min = 0.01;
const float psi_max = 2*M_PI;
const float psi_min = 0.0;
const float delta_d_max = 0.4;
const float delta_d_min = -0.4;
const float d_max = 1.0;
const float d_min = -1.0;
const float delta_dz_up= 0.0775;
const float delta_dz_lo = -0.0925;
// -------------- Declaración dimencion matrices MPC ----------------------------
const int nx = 3;
const int ny = 2;
const int nu = 2;

class MpcLlcNode : public rclcpp::Node
{
public:
    MpcLlcNode() : Node("mpc_llc")
    {     
        
        //--------- Parámetros del LLC MPC-------------------//
        this-> declare_parameter("Ts", 100.0);

        this-> declare_parameter("W_psi", 3.0);
        this-> declare_parameter("W_u", 5.0);
        this-> declare_parameter("W_dL", 7.0);
        this-> declare_parameter("W_dR", 1.0);
        this-> declare_parameter("NC", 5);
        this-> declare_parameter("NP", 5);

        //----------- Parametros Model --------------------------//

        this-> declare_parameter("Xu6", -0.003148);
        this-> declare_parameter("Xu7", 0.0810014);
        this-> declare_parameter("Xr10", -0.0129643);
        this-> declare_parameter("Xr11", -0.0110386);
        this-> declare_parameter("Xr12", 0.0);
        this-> declare_parameter("Xr13", 0.1717909);

        //--------- Obtener parametros LLC MPC-------------------//

        Ts = this->get_parameter("Ts").as_double()/1000.0;
        W_psi = this->get_parameter("W_psi").as_double();
        W_u = this->get_parameter("W_u").as_double();
        W_dL = this->get_parameter("W_dL").as_double();
        W_dR = this->get_parameter("W_dR").as_double();
        NC = this->get_parameter("NC").as_int();
        NP = this->get_parameter("NP").as_int();

        Xu6 = this->get_parameter("Xu6").as_double();
        Xu7 = this->get_parameter("Xu7").as_double();
        Xr10 = this->get_parameter("Xr10").as_double();
        Xr11 = this->get_parameter("Xr11").as_double();
        Xr12 = this->get_parameter("Xr12").as_double();
        Xr13 = this->get_parameter("Xr13").as_double();

        //----------- Crear Matrices MPC ------------------------//
        Q.resize(NP*ny, NP*ny);
        R.resize(NC*nu, NC*nu);
        Y_ref.resize(NP*ny);
        Y.resize(NP*ny);
        Ar.resize(2*NC*nu, NC*nu);
        Br.resize(2*NC*nu);
        Ymax.resize(ny*NP);
        Ymin.resize(ny*NP);
        Umax.resize(nu*NC);
        Umin.resize(nu*NC);

        Q.setZero();
        R.setZero();
        Y_ref.setZero();
        Y.setZero();
        Ar.setZero();
        Br.setZero();
        Ymax.setZero();
        Ymin.setZero();
        Umax.setZero();
        Umin.setZero();

        fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
        fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
        fillArMatrix(Ar, nu, NC);
        fillBrVector(Br, nu, NC);
        fillYRestrictions(Ymin, Ymax, ny, NP);
        fillURestrictions(Umin, Umax, nu, NC);
        
        // Print the resulting matrix o vector
        //std::cout << "A matrix:\n" << Umin << std::endl;

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcLlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer",rclcpp::SensorDataQoS(), std::bind(&MpcLlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<geometry_msgs::msg::Vector3>(
            "/control/reference_llc", 1, std::bind(&MpcLlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&MpcLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/control/pwm_value_ifac",
                10);
        /*timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcLlcNode::calculateLowLevelController, this), cb_group_obs_);
        */
        RCLCPP_INFO(this->get_logger(), "Low Level Controller MPC Node has been started.");
    	
    }

private:
    void calculateLowLevelController()
    {
        if(armed==false){
            Y_ref.setZero();
            Y.setZero();
            count=0;
            dL=0.0;
            dR=0.0;
            Br.segment(0, 2 * nu) << 0.0, 
                                     0.0, 
                                     0.0, 
                                     0.0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;
                sig_u = 0.0;
                sig_r = 0.0;
            }
        }else{
            if(count > 5){
                //auto start = std::chrono::high_resolution_clock::now();
                auto msg = asv_interfaces::msg::PwmValues();

                float u_hat_i;
                float r_hat_i;
                float psi_hat_i;
                float sig_u_i;
                float sig_r_i;
                VectorXd Y_ref_i;
                Y_ref_i.setZero();
                Br.segment(0, 2 * nu) << -delta_d_min - dL, 
                                delta_d_max + dL, 
                                -delta_d_min - dR, 
                                delta_d_max + dR;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Y_ref_i=Y_ref;
                    u_hat_i =u_hat;
                    r_hat_i= r_hat;
                    psi_hat_i = psi_hat;
                    sig_u_i=sig_u;
                    sig_r_i=sig_r;
                }

                //----------------- Codigo del MPC-------------------//

                
                //----------------- Asignar valores PWM -------------//
                dL=0.5;
                dR=0.5;
                // Publish pwms
                msg.t_left=400 * dL + 1500;
                msg.t_righ=400 * dR + 1500;
                if(msg.t_left<1500){
                    msg.t_left=1500;
                }else if (msg.t_left > 1900) {
                    msg.t_left=1900;
                }
                if(msg.t_righ<1500){
                    msg.t_righ=1500;
                }else if (msg.t_righ > 1900) {
                    msg.t_righ=1900;
                }
                    
                publisher_pwm->publish(msg);
                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start;
                // double miliseconds = elapsed.count()*1000;
                // Imprime el tiempo con dos decimales fijos
                // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
            }else{
                auto msg = asv_interfaces::msg::PwmValues();
                msg.t_left= 1500;
                msg.t_righ= 1500; 
                count=count+1;
                publisher_pwm->publish(msg);
            }
        }        
    }


    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_hat = msg->velocity.x;
            r_hat = msg->velocity.z;
            psi_hat = msg->point.z;
            sig_u = msg->disturbances.x;
            sig_r = msg->disturbances.z;
        }
    }

    void callbackVelReference(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            /*u_ref = msg->x;
            r_ref = msg->y;
            psi_ref = msg->z;*/
        
            // Fill the y_ref vector
            for (int i = 0; i < ny * NP; i += ny) {
                Y_ref.segment(i, ny) << msg->z, msg->x;
            }
        }
    }

    // Función para calcular la salida del filtro de derivación
    float derivationFilter(float input, std::vector<float>& memory, float a, float b){
        float output = (a * memory[0]) + b * (input - memory[1]);
        // Actualizar memoria para la próxima iteración
        memory[0] = output;
        memory[1] = input;
        return output;
    }  

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    void fillSquareMatrix(MatrixXd &M, int const size, int const sub_size, float const W1, float const W2){
        // Loop to fill the matrix Q with diagonal blocks
        for (int i = 0; i < size; ++i){ 
            // Create the diagonal 
            VectorXd diag_block(sub_size);
            diag_block << W1, W2;      // Adjust based on the actual ny value and the values you want to set
            
            // Set the diagonal block in the matrix Q       
            M.block(i * sub_size, i * sub_size, sub_size, sub_size) = diag_block.asDiagonal();     
        }
    }

    void fillArMatrix(MatrixXd &A, int const nu, int const NC){
        // First loop
        for (int i = 1; i <= nu * NC; ++i) {
            A(i * 2 - 2, i - 1) = -1; // Adjust for 0-based index
            A(i * 2 - 1, i - 1) = 1;  // Adjust for 0-based index
        }
        // Second loop
        for (int i = nu + 1; i <= nu * NC; ++i) {
            A((i * 2) - 2, (i - nu) - 1) = 1;  // Adjust for 0-based index
            A((i * 2) - 1, (i - nu) - 1) = -1; // Adjust for 0-based index
        }
    }

    void fillBrVector(VectorXd &B, int const nu, int const NC){
        for (int i = 2 * nu; i < 2 * nu * NC; i += 2 * nu) {
            B.segment(i, 4) << -delta_d_min, 
                                delta_d_max, 
                               -delta_d_min, 
                                delta_d_max;
        }
    }

    void fillYRestrictions(VectorXd &Y_min, VectorXd &Y_max, int const ny, int const NP){
        for (int i = 0; i < ny * NP; i += ny) {
            Y_min.segment(i, ny) << psi_min, u_min;
            Y_max.segment(i, ny) << psi_max, u_max;
        }
    }

    void fillURestrictions(VectorXd &U_min, VectorXd &U_max, int const nu, int const NC){
        for (int i = 0; i < nu * NC; i += nu) {
            U_min.segment(i, nu) << d_max, d_max;
            U_max.segment(i, nu) << d_min, d_min;
        }
    }


    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "W_psi"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_psi = param.as_double();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_u"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_u = param.as_double();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_dL"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_dL = param.as_double();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_dR"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_dR = param.as_double();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "NC"){
                if(param.as_int() >= 1 and param.as_int() < 30){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NC = param.as_int();
                    Ar.resize(2*NC*nu, NC*nu);
                    Br.resize(2*NC*nu);
                    R.resize(NC*nu, NC*nu);
                    Umax.resize(nu*NC);
                    Umin.resize(nu*NC);
                    Ar.setZero();
                    Br.setZero();
                    R.setZero();
                    Umax.setZero();
                    Umin.setZero();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
                    fillArMatrix(Ar, nu, NC);
                    fillBrVector(Br, nu, NC);
                    fillURestrictions(Umin, Umax, nu, NC);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "NP"){
                if(param.as_int() >= 1 and param.as_int() < 30){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NP = param.as_int();
                    Y_ref.resize(NP*ny);
                    Y.resize(NP*ny);
                    Q.resize(NP*ny, NP*ny);
                    Ymax.resize(ny*NP);
                    Ymin.resize(ny*NP);
                    Q.setZero();
                    Y_ref.setZero();
                    Y.setZero();
                    Ymax.setZero();
                    Ymin.setZero();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
                    fillYRestrictions(Ymin, Ymax, ny, NP);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }
    /******/
    bool armed = false;  // PILAS DEBE SER FALSE PARA IMPLEMENTAR
    /******/
    float sig_u, sig_r, u_hat, r_hat, psi_hat;
    int count=0;
    //------Params-------//
    float Ts;  
    float dL, dR;
    float Xu6, Xu7, Xr10, Xr11 ,Xr12, Xr13;  
    /*Parámetros del controlador MPC*/
    float W_psi, W_u, W_dL, W_dR;
    int NC, NP;
    /* Definicion Matrices y Vectores*/
    MatrixXd Q;
    MatrixXd R;
    MatrixXd Ar;
    VectorXd Br;
    VectorXd Y_ref;
    VectorXd Y;
    VectorXd Ymax;
    VectorXd Ymin;
    VectorXd Umax;
    VectorXd Umin;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcLlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}