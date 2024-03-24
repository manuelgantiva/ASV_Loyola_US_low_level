#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer


#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

// Declaración de contantes
const int PWMMAX = 1900;
const int PWMMIN = 1900;

const float IGu_MAX = 0.0381065;
const float IGu_MIN = 0.0;

const float IGr_MAX = 0.0507984;
const float IGr_MIN = -0.0507984;

class IfacLlcNode : public rclcpp::Node
{
public:
    IfacLlcNode() : Node("ifac_llc")
    {     
        
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("sm_gain_ku", 2.0);
        this-> declare_parameter("sm_gain_ki", 2.0);
        this-> declare_parameter("sm_gain_kpsi", 1.0);
        this-> declare_parameter("sm_gain_kr", 4.0);
        this-> declare_parameter("taud", 350); // Taud = #*Ts Est es #
        this-> declare_parameter("IGr_en", 1.0);
        this-> declare_parameter("Su_en", 1.0);

        this-> declare_parameter("Xu6", -0.0166263484863104);
        this-> declare_parameter("Xu7", 0.0592216770505099);
        this-> declare_parameter("Xr11", 0.0127391511137106);
        this-> declare_parameter("Xr13", 0.190555832330969);
        
    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        sm_gain_ku = this->get_parameter("sm_gain_ku").as_double();
        sm_gain_ki = this->get_parameter("sm_gain_ki").as_double();
        sm_gain_kpsi = this->get_parameter("sm_gain_kpsi").as_double();
        sm_gain_kr = this->get_parameter("sm_gain_kr").as_double();
        taud = this->get_parameter("taud").as_int();
        IGr_en = this->get_parameter("IGr_en").as_double();
        Su_en = this->get_parameter("Su_en").as_double();

        Xu6 = this->get_parameter("Xu6").as_double();
        Xu7 = this->get_parameter("Xu7").as_double();
        Xr11 = this->get_parameter("Xr11").as_double();
        Xr13 = this->get_parameter("Xr13").as_double();
        
        memory_u.assign(4, 0.0);
        memory_r.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&IfacLlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer",rclcpp::SensorDataQoS(), std::bind(&IfacLlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<geometry_msgs::msg::Vector3>(
            "/control/reference_llc", 1, std::bind(&IfacLlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&IfacLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/control/pwm_value_ifac",
                rclcpp::SensorDataQoS());

        publisher_IGu = this-> create_publisher<geometry_msgs::msg::Vector3>("/control/IGu_ifac",1);
        publisher_IGr = this-> create_publisher<geometry_msgs::msg::Vector3>("/control/IGr_ifac",1);

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&IfacLlcNode::calculateLowLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller IFAC Node has been started.");
    	
    }

private:
    void calculateLowLevelController()
    {
        if(armed==false){
            c_ref=0;
            memory_u.assign(4, 0.0);
            memory_r.assign(4, 0.0);
            count=0;
            integral_error=0;
        }else{
            //auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::PwmValues();

            auto msg_Igu = geometry_msgs::msg::Vector3();
            auto msg_Igr = geometry_msgs::msg::Vector3();

            float u_hat_i;
            float r_hat_i;
            float psi_hat_i;
            float sig_u_i;
            float sig_r_i;

            float u_ref_i;
            float r_ref_i;
            float u_dot_ref_i;
            float r_dot_ref_i;
            float psi_ref_i;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat_i=u_hat;
                r_hat_i=r_hat;
                psi_hat_i=psi_hat;
                sig_u_i=sig_u;
                sig_r_i=sig_r;
                u_ref_i=u_ref;
                r_ref_i=r_ref;
                u_dot_ref_i=u_dot_ref;
                r_dot_ref_i=r_dot_ref;
                psi_ref_i=psi_ref;
            }

            float error = (u_hat_i-u_ref_i);
            integral_error += error;
            msg_Igu.x = Ts*u_dot_ref_i;
            msg_Igu.y = Ts*sm_gain_ku*error;
            msg_Igu.z = Ts*sm_gain_ki*integral_error;
            float sg = Su_en*Ts*sig_u_i;

            float IG_u = msg_Igu.x - msg_Igu.y - msg_Igu.z - sg;
            //float IG_u = Ts*(u_dot_ref_i-sm_gain_ku*(u_hat_i-u_ref_i)-sig_u_i);

            float c_ref=r_ref_i-sm_gain_kpsi*(psi_hat_i-psi_ref_i);
            msg_Igr.x = sm_gain_kr*(r_hat_i-c_ref);
            msg_Igr.y = sm_gain_kpsi*(r_hat_i-r_ref_i);
            msg_Igr.z = r_dot_ref_i;
            float IG_r = IGr_en*Ts*(r_dot_ref_i-msg_Igr.x-msg_Igr.y-sig_r_i);

            if(IG_u==0 && IG_r==0){
                msg.t_left=1500;
                msg.t_righ=1500;  
            }else{
                if(IG_u<IGu_MIN){
                    IG_u=IGu_MIN;
                }else if (IG_u > IGu_MAX) {
                    IG_u=IGu_MAX;
                }

                if(IG_r<IGr_MIN){
                    IG_r=IGr_MIN;
                }else if (IG_r > IGr_MAX) {
                    IG_r=IGr_MAX;
                }

                double t0 = -4 * Xr11 * Xr11;
                double t1 = -(4 * Xr11 * Xr11 * Xu7 / Xu6) - (4 * Xr11 * Xr13);
                double t2 = (4 * Xr11 * Xr11 * IG_u / Xu6) - (4 * Xr11 * Xr13 * Xu7 / Xu6) - (Xr13 * Xr13);
                double t3 = (4 * Xr11 * Xr13 * IG_u / Xu6) - (Xr13 * Xr13 * Xu7 / Xu6);
                double t4 = (Xr13 * Xr13 * IG_u / Xu6) - (IG_r * IG_r);


                vector<double> yr = solveY(t0, t1, t2, t3, t4);
                vector<double> res;

                for (double yi : yr) {
                    if (yi >= 0 && yi <= 1.13) {
                        res.push_back(yi);
                    }
                }

                double xf, yf;
                for (double yi : res) {
                    double xi = solveX(yi, IG_r, Xr11, Xr13);
                    xf = xi;
                    yf = yi;
                }

                double L, R;
                L = ((2 * yf + xf) / 2);
                if (L > 0) {
                    L = L + 0.0775;
                } else {
                    L = L - 0.0925;
                }

                R = ((2 * yf - xf) / 2);
                if (R > 0) {
                    R = R + 0.0775;
                } else {
                    R = R - 0.0925;
                }

                // Publish pwms
                msg.t_left=400 * L + 1500;
                msg.t_righ=400 * R + 1500;
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
                
            }
            if(count < 5){
                msg.t_left= 1500;
                msg.t_righ= 1500; 
                count=count+1;
            }
            publisher_pwm->publish(msg);
            publisher_IGu->publish(msg_Igu);
            publisher_IGr->publish(msg_Igr);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // double miliseconds = elapsed.count()*1000;
            // Imprime el tiempo con dos decimales fijos
            // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
        }        
    }

    vector<double> filterRoots(const VectorXcd& roots) {
        vector<double> realRoots;
        for (int i = 0; i < roots.size(); ++i) {
            if (roots[i].imag() == 0) {
                double realPart = roots[i].real();
                realRoots.push_back(realPart);
            }
        }
        return realRoots;
    }

    vector<double> solveY(double t0, double t1, double t2, double t3, double t4) {
        VectorXcd coefficients(5);
        coefficients << t4, t3, t2, t1, t0;

        Eigen::PolynomialSolver<complex<double>, Eigen::Dynamic> solver;
        solver.compute(coefficients);

        return filterRoots(solver.roots());
    }

    double solveX(double y, double b, double c11, double c13) {
        double n = b / ((c11 * y) + (c13 / 2));
        return n;
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
   
        float u_dot = derivationFilter(msg->x, memory_u, a, b);
        float r_dot = derivationFilter(msg->y, memory_r, a, b);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_ref = msg->x;
            r_ref = msg->y;
            u_dot_ref = u_dot;
            r_dot_ref = r_dot;
            psi_ref = msg->z;
        }
    }

    // Función para calcular la salida del filtro de derivación
    float derivationFilter(float input, std::vector<float>& memory, float a, float b){
        //float a1 = 0.0;
        //float b0 = 1.0;
        //float output = (b0 * input - f_diff * memory[0] + f_diff * memory[1] - a1 * memory[2] + f_diff * memory[3]) / f_diff;
        
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

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "sm_gain_ku"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_ku = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "sm_gain_ki"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_ki = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "sm_gain_kpsi"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_kpsi = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "sm_gain_kr"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_kr = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "taud"){
                if(param.as_int() >= 0 and param.as_int() < 500){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    taud = param.as_int();
                    a=(taud*Ts)/(taud*Ts+Ts);
                    b=1/(taud*Ts+Ts);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "IGr_en"){
                if(param.as_double() == 0.0 or param.as_double() == 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    IGr_en = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be 0.0 or 1.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Su_en"){
                if(param.as_double() == 0.0 or param.as_double() == 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Su_en = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be 0.0 or 1.0");
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

    bool armed = false;
    float u_hat, psi_hat, r_hat, sig_u, sig_r, u_ref, psi_ref, r_ref, u_dot_ref, r_dot_ref;
    float c_ref;
    int count=0;
    float integral_error=0;
    //------Params-------//
    float Ts;  
    /*Parámetros del controlador Sliding Modes*/
    float sm_gain_ku; /*Ganancia del  controlador Sliding Modes (surge)*/
    float sm_gain_ki; /*Ganancia del  controlador Sliding Modes (surge constant integrative)*/
    float sm_gain_kpsi; /*Ganancia 1 del controlador Sliding Modes (yaw)*/
    float sm_gain_kr; /*Ganancia 2 del controlador Sliding Modes (yaw)*/
    
    float taud; /*Constante tau del filtro derivativo*/
    float a ,b; /*Constantes del filtro derivativo*/
    float IGr_en, Su_en; /*Enable IGr y Sigmas surge*/

    float Xu6, Xu7, Xr11 , Xr13;  

    std::vector<float> memory_u; // Memoria para mantener los valores anteriores
    std::vector<float> memory_r; // Memoria para mantener los valores anteriores

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_IGu;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_IGr;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IfacLlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}