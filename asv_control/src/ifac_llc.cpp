#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/reference_llc.hpp"


#include <cmath>
#include <thread>
#include <vector>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

// Declaración de contantes
const int PWMMAX = 1900;
const int PWMMIN = 1100;

class IfacLlcNode : public rclcpp::Node
{
public:
    IfacLlcNode() : Node("ifac_llc")
    {     
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("ku", 2.0);
        this-> declare_parameter("kpsi", 1.0);
        this-> declare_parameter("kr", 4.0);
        this-> declare_parameter("taud", 350); // Taud = #*Ts Est es #
        this-> declare_parameter("Sat", 0.3); // Coeficiente de saturacion
        this-> declare_parameter("delta_pwm", 250); // Taud = #*Ts Est es #
        
        this-> declare_parameter("mf", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("mr", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("df", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("dr", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        this-> declare_parameter("IGumax_ff", 0.17794);
        this-> declare_parameter("IGumax_rf", 0.08897),
        this-> declare_parameter("IGumin_rf", -0.07331);
        this-> declare_parameter("IGrmax_ff", 0.14128);
        this-> declare_parameter("IGrmax_rf", 0.22898);

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        my_id = (this->get_parameter("my_id").as_string());
        
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        sm_gain_ku = this->get_parameter("ku").as_double();
        sm_gain_kpsi = this->get_parameter("kpsi").as_double();
        sm_gain_kr = this->get_parameter("kr").as_double();
        taud = this->get_parameter("taud").as_int();
        Sat = this->get_parameter("Sat").as_double();
        delta_pwm = this->get_parameter("delta_pwm").as_int();

        mf = this->get_parameter("mf").as_double_array();
        mr = this->get_parameter("mr").as_double_array();
        df = this->get_parameter("df").as_double_array();
        dr = this->get_parameter("dr").as_double_array();

        IGumax_ff = this->get_parameter("IGumax_ff").as_double();
        IGumax_rf = this->get_parameter("IGumax_rf").as_double();
        IGumin_rf = this->get_parameter("IGumin_rf").as_double();
        IGrmax_ff = this->get_parameter("IGrmax_ff").as_double();
        IGrmax_rf = this->get_parameter("IGrmax_rf").as_double();

        Dz_2  = this->get_parameter("Dz_up").as_double();
        Dz_1 = this->get_parameter("Dz_down").as_double();
        p = 1 - Dz_2;
        q = -1 - Dz_1;
        
        memory_u.assign(4, 0.0);
        memory_r.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&IfacLlcNode::calculateLowLevelController, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&IfacLlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&IfacLlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/" + my_id + "/control/reference_llc", 1, std::bind(&IfacLlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&IfacLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/" + my_id + "/control/pwm_value_ifac",
                1);

        publisher_IG = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/IG_ifac",1);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller IFAC Node in %s has been started.", my_id.c_str());
    	
    }

private:
    void calculateLowLevelController()
    {
        if(armed==false){
            c_ref=0;
            memory_u.assign(4, 0.0);
            memory_r.assign(4, 0.0);
            count=0;
            IGu_prev = 0.0;
            IGr_prev = 0.0;
            PWM_left_ant = 1500;
            PWM_right_ant = 1500;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat = 0.0;
                r_hat= 0.0;
                u_ref = 0.5;
                r_ref = 0.0;
                flag_ref = false;
            }
        }else{
            //auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::PwmValues();
            auto msg_Igu = geometry_msgs::msg::Vector3();
            auto msg_Igr = geometry_msgs::msg::Vector3();
            auto msg_Ig = geometry_msgs::msg::Vector3();
            float zone;

            if(count > 8){
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
                    if(flag_ref){
                        psi_ref_i= psi_ref;
                    }else{
                        psi_ref_i= psi_hat;
                    }    
                }

                float error = (u_hat_i-u_ref_i);
                msg_Igu.x = u_dot_ref_i;
                msg_Igu.y = sm_gain_ku*error;

                float c_ref=r_ref_i-sm_gain_kpsi*(psi_hat_i-psi_ref_i);
                msg_Igr.x = sm_gain_kr*(r_hat_i-c_ref);
                msg_Igr.y = sm_gain_kpsi*(r_hat_i-r_ref_i);
                msg_Igr.z = r_dot_ref_i;
                float IG_u;
                float IG_r;

                IG_u = msg_Igu.x - msg_Igu.y - msg_Igu.z - (sig_u_i);
                IG_r = msg_Igr.z - msg_Igr.x - msg_Igr.y - (sig_r_i);
                

                // New Code
                float m, d;

                if(IG_u>IGumax_ff){
                    IG_u=IGumax_ff;
                }

                if(IG_u<IGumin_rf){
                    IG_u=IGumin_rf;
                }

                if(IG_r>IGrmax_rf){
                    IG_r=IGrmax_rf;
                }

                if(IG_r<-IGrmax_rf){
                    IG_r=-IGrmax_rf;
                }

                if(IG_u>IGumax_ff){
                    IG_u=IGumax_ff;
                }

                if (IG_u - IGu_prev > (IGumax_ff - IGumin_rf) * Sat) {
                    IG_u = IGu_prev + (IGumax_ff - IGumin_rf) * Sat;
                } else if (IGu_prev - IG_u > (IGumax_ff - IGumin_rf) * Sat) {
                    IG_u = IGu_prev - (IGumax_ff - IGumin_rf) * Sat;
                }

                if (IG_r - IGr_prev > (2*IGrmax_rf) * Sat) {
                    IG_r = IGr_prev + (2*IGrmax_rf) * Sat;
                } else if (IGr_prev - IG_r > (2*IGrmax_rf) * Sat) {
                    IG_r = IGr_prev - (2*IGrmax_rf) * Sat;
                }
                
                if(IG_u>IGumax_rf){
                    // Zona Roja
                    m = mf[0]+mf[1]*IG_u+mf[2]*IG_r+mf[3]*IG_u*IG_u+mf[4]*IG_u*IG_r+mf[5]*IG_r*IG_r;
                    d = df[0]+df[1]*IG_u+df[2]*IG_r+df[3]*IG_u*IG_u+df[4]*IG_u*IG_r+df[5]*IG_r*IG_r;
                    zone = 0;
                }else if(IG_r>IGrmax_ff){
                    // Zona Azul
                    m = mr[0]+mr[1]*IG_u+mr[2]*IG_r+mr[3]*IG_u*IG_u+mr[4]*IG_u*IG_r+mr[5]*IG_r*IG_r;
                    d = dr[0]+dr[1]*IG_u+dr[2]*IG_r+dr[3]*IG_u*IG_u+dr[4]*IG_u*IG_r+dr[5]*IG_r*IG_r;
                    zone = 1;
                }else if(IG_r<-IGrmax_ff){
                    // Zona Verde
                    m = mr[0]+mr[1]*IG_u-mr[2]*IG_r+mr[3]*IG_u*IG_u-mr[4]*IG_u*IG_r+mr[5]*IG_r*IG_r;
                    d = -dr[0]-dr[1]*IG_u+dr[2]*IG_r-dr[3]*IG_u*IG_u+dr[4]*IG_u*IG_r-dr[5]*IG_r*IG_r;
                    zone = -1;
                }else{
                    // Zona Roja
                    m = mf[0]+mf[1]*IG_u+mf[2]*IG_r+mf[3]*IG_u*IG_u+mf[4]*IG_u*IG_r+mf[5]*IG_r*IG_r;
                    d = df[0]+df[1]*IG_u+df[2]*IG_r+df[3]*IG_u*IG_u+df[4]*IG_u*IG_r+df[5]*IG_r*IG_r;
                    zone = 0;
                    if((m<=0.5*d) || (m<=-0.5*d)){
                        if(IG_r>=0){
                            //Zona Azul
                            m = mr[0]+mr[1]*IG_u+mr[2]*IG_r+mr[3]*IG_u*IG_u+mr[4]*IG_u*IG_r+mr[5]*IG_r*IG_r;
                            d = dr[0]+dr[1]*IG_u+dr[2]*IG_r+dr[3]*IG_u*IG_u+dr[4]*IG_u*IG_r+dr[5]*IG_r*IG_r;
                            zone = 1;
                        }else{
                            //Zona Verde
                            m = mr[0]+mr[1]*IG_u-mr[2]*IG_r+mr[3]*IG_u*IG_u-mr[4]*IG_u*IG_r+mr[5]*IG_r*IG_r;
                            d = -dr[0]-dr[1]*IG_u+dr[2]*IG_r-dr[3]*IG_u*IG_u+dr[4]*IG_u*IG_r-dr[5]*IG_r*IG_r;
                            zone = -1;
                        }
                    }
                }

                double L, R;
                L = ((2 * m + d) / 2);
                R = ((2 * m - d) / 2);

                if (L > 0) {
                    L = L + Dz_2;
                } else if (L < 0){
                    L = L + Dz_1;
                }

                if (R > 0) {
                    R = R + Dz_2;;
                } else if (R < 0){
                    R = R + Dz_1;
                }

                // Publish pwms

                msg.t_left = denormalizationPwm(L, PWM_left_ant);
                msg.t_righ = denormalizationPwm(R, PWM_right_ant);;

                msg_Ig.x = IG_u;
                msg_Ig.y = IG_r;
                msg_Ig.z = zone;
                publisher_pwm->publish(msg);
                publisher_IG->publish(msg_Ig);
                IGu_prev = IG_u;
                IGr_prev = IG_r;

                PWM_left_ant = msg.t_left;
                PWM_right_ant = msg.t_righ;
            }else{
                msg.t_left= 1500;
                msg.t_righ= 1500; 
                count=count+1;
                publisher_pwm->publish(msg);
                publisher_IG->publish(msg_Ig);
            }
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // double miliseconds = elapsed.count()*1000;
            // Imprime el tiempo con dos decimales fijos
            // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
        }        
    }

    uint16_t denormalizationPwm(double delta, uint16_t pwm_ant) {
        int resultado = static_cast<int>(400 * delta) + 1500;
    
        if (resultado < 1100) {
            resultado = 1100;
        } else if (resultado > 1900) {
            resultado = 1900;
        }

        int delta_diff = resultado - pwm_ant;
        if (delta_diff > delta_pwm) {
            resultado = pwm_ant + delta_pwm;
        } else if (delta_diff < -delta_pwm) {
            resultado = pwm_ant - delta_pwm;
        }

        return static_cast<uint16_t>(resultado);
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

    void callbackVelReference(const asv_interfaces::msg::ReferenceLlc::SharedPtr msg)
    {
        const auto &refs = msg->references;
        const auto &vec = refs[0];
        float u_dot = derivationFilter(vec.x, memory_u, a, b);
        float r_dot = derivationFilter(vec.y, memory_r, a, b);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_ref = vec.x;
            r_ref = vec.y;
            u_dot_ref = u_dot;
            r_dot_ref = r_dot;
            psi_ref = vec.z;
            flag_ref = true;
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
            if (param.get_name() == "ku"){
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
            if (param.get_name() == "kpsi"){
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
            if (param.get_name() == "kr"){
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
            if (param.get_name() == "Sat"){
                if(param.as_double() >= 0.0 or param.as_double() <= 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Sat = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0.0 - 1.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "delta_pwm"){
                if(param.as_int() >= 0 and param.as_int() <= 500){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    delta_pwm = param.as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-500");
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

    uint16_t PWM_left_ant = 1500, PWM_right_ant = 1500;
    int delta_pwm;
    bool armed = false, flag_ref = false;
    float u_hat = 0, psi_hat = 0, r_hat = 0, sig_u = 0, sig_r = 0, u_ref = 0.5, psi_ref = 0, r_ref = 0, u_dot_ref = 0, r_dot_ref = 0;
    float c_ref;
    int count=0;
    float IGu_prev = 0.0, IGr_prev = 0.0;
    //------Params-------//
    float Ts;  
    /*Parámetros del controlador Sliding Modes*/
    float sm_gain_ku; /*Ganancia del  controlador Sliding Modes (surge)*/
    float sm_gain_kpsi; /*Ganancia 1 del controlador Sliding Modes (yaw)*/
    float sm_gain_kr; /*Ganancia 2 del controlador Sliding Modes (yaw)*/
    
    float taud; /*Constante tau del filtro derivativo*/
    float a ,b; /*Constantes del filtro derivativo*/
    float Sat; /*Coeficientes de saturacion*/

    std::vector<double> mf, mr, df, dr;


    float IGumax_ff, IGumax_rf, IGumin_rf;
    float IGrmax_ff, IGrmax_rf;
    float Dz_1, Dz_2, p , q;

    std::vector<float> memory_u; // Memoria para mantener los valores anteriores
    std::vector<float> memory_r; // Memoria para mantener los valores anteriores

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_IG;

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