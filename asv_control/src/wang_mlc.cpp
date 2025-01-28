#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <cmath>
#include <thread>
// #include "asv_library/curvas_sim.h"
#include "asv_library/curvas_alamillo.h"

using namespace std;

using std::placeholders::_1;

// Declaración de contantes

class WangMlcNode : public rclcpp::Node
{
public:
    WangMlcNode() : Node("wang_mlc")
    {     
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("delta_SGLOS", 8.0);
        this-> declare_parameter("k_u_tar", 2.0);
        this-> declare_parameter("taud", 15.0); // Taud = #*Ts Est es #
        this-> declare_parameter("path_d", 0); // path_d = #Path deseado #
        this-> declare_parameter("flag", true); // path_d = #Path deseado #

        my_id = (this->get_parameter("my_id").as_string());    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        delta_SGLOS = this->get_parameter("delta_SGLOS").as_double();
        k_u_tar = this->get_parameter("k_u_tar").as_double();
        taud = this->get_parameter("taud").as_double();
        path_d  = this->get_parameter("path_d").as_int();
        flag  = this->get_parameter("flag").as_bool();

        memory_psi.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);
        w = 0.0;
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&WangMlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&WangMlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_mlc", 1, std::bind(&WangMlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&WangMlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_llc = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/reference_llc",1);
        publisher_error = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/error_mlc",1);

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&WangMlcNode::calculateMidLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Mid Level Controller Wang Node in %s has been started.", my_id.c_str());
    	
    }

private:
    void calculateMidLevelController()
    {
        if(armed==false){
            memory_psi.assign(4, 0.0);
            count=0;
            w=0.0;
            laps=0;
        }else{
            if(count > 7){
                //auto start = std::chrono::high_resolution_clock::now();
                auto msg = geometry_msgs::msg::Vector3();
                auto msg_e = geometry_msgs::msg::Vector3();

                float x_hat_i;
                float y_hat_i;
                float v_hat_i;
                float psi_hat_i;
                float u_d_i;
                float xe, ye;
                float xp_i, yp_i;
                float dxp_i, dyp_i;
                float psip_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    x_hat_i = x_hat;
                    y_hat_i = y_hat;
                    v_hat_i=v_hat;
                    psi_hat_i=psi_hat;
                    u_d_i=u_d;
                }
                Target p_i = currentTarget(w);
                xp_i=p_i.xp;
                yp_i=p_i.yp;
                dxp_i=p_i.dxp;
                dyp_i=p_i.dyp;
                psip_i = atan2(dyp_i, dxp_i);

                psi_hat_i = normalizeAngle(psi_hat_i);

                xe = (x_hat_i - xp_i)*cos(psip_i) + (y_hat_i - yp_i)*sin(psip_i);
                ye = -1*(x_hat_i - xp_i)*sin(psip_i) + (y_hat_i - yp_i)*cos(psip_i);

                msg_e.x = xe;
                msg_e.y = ye;
                msg_e.z = w;

                float k1_i = u_d_i / delta_SGLOS;
                float u_ref = k1_i * std::sqrt(delta_SGLOS*delta_SGLOS + ye*ye);
                float b_ref = 0.0;
                if(flag){
                    b_ref = atan2(v_hat_i, u_ref);
                }else{
                    b_ref = -1*atan2(v_hat_i, u_ref);
                }
                float psi_ref = psip_i - b_ref - atan2(ye, delta_SGLOS);
                float U_ref = std::sqrt(u_ref*u_ref + v_hat_i*v_hat_i);
                float u_tar = k_u_tar*xe + U_ref*cos(psi_hat_i-psip_i+b_ref);
                float w_dot = u_tar / (std::sqrt(dxp_i*dxp_i + dyp_i*dyp_i));

                w += Ts*w_dot;
                // Corrijo el angulo de referencia teniendo en cuenta las vueltas sobre la trayectoria
                if(psi_ref<0){
                    psi_ref=psi_ref+(2*M_PI);
                }

                if(armed_act==false){
                    psi_ant = psi_ref;
                    laps = 0;
                }else{
                    if((psi_ref - psi_ant) > M_PI){
                        laps = laps - 1;
                    }else if((psi_ref - psi_ant) < -M_PI){
                        laps = laps + 1;
                    }
                    psi_ant=psi_ref;
                    psi_ref += 2*M_PI*laps;
                }
                        
                float r_ref = derivationFilter(psi_ref, memory_psi, a, b);

                float r_ref_max = 0.8; //0.6
                if(r_ref > r_ref_max){
                    r_ref = r_ref_max;
                }else if(r_ref < -r_ref_max){
                    r_ref = -r_ref_max;
                }

                if(u_ref > 1.5){
                    u_ref = 1.5;
                }else if(u_ref < -1.5){
                    u_ref = -1.5;
                }
                
                msg.x = u_ref;
                msg.y = r_ref;
                msg.z = psi_ref;

                publisher_llc->publish(msg);
                publisher_error->publish(msg_e);
                armed_act = armed;
                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start;
                // double miliseconds = elapsed.count()*1000;
                // Imprime el tiempo con dos decimales fijos
                // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
            }else{
                auto msg = geometry_msgs::msg::Vector3();
                msg.x = 0.0;
                msg.y = 0.0;
                msg.z = 0.0; 
                count=count+1;
                publisher_llc->publish(msg);
            }
        }       
    }

    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat = msg->point.x;
            y_hat = msg->point.y;
            v_hat = msg->velocity.y;
            psi_hat = msg->point.z;
        }
    }

    void callbackVelReference(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_d = msg->data;
        }
    }

    // Función para calcular la salida del filtro de derivación
    float derivationFilter(float input, std::vector<float>& memory, float a, float b){
        float output = (a * memory[0]) + b * (input - memory[1]);

        // RCLCPP_INFO(this->get_logger(), "current left: %f and previous right: %f", input, memory[1]);
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

    Target currentTarget(float w){
        Target result;
        switch(path_d) {
            case 0:
                result.xp = -w;
                result.yp = 0;
                result.dxp = -1;
                result.dyp = 0;
                break;
            case 1:
                // result = curva_sim_2_6(w);
                result = curva_ala_1_2(w);
                break;
            case 2:
                // result = curva_sim_2_8(w);
                result = curva_ala_1_4(w);
                break;
            case 3:
                // result = curva_sim_2_10(w);
                result = curva_ala_1_6(w);
                break;
            case 4:
                // result = curva_sim_3_6(w);
                result = curva_ala_2_2(w);
                break;
            case 5:
                // result = curva_sim_3_8(w);
                result = curva_ala_2_4(w);
                break;
            case 6:
                // result = curva_sim_3_10(w);
                result = curva_ala_2_6(w);
                break;
            case 7:
                result = curva_ala_3_1(w);
                break;
            case 8:
                result = curva_ala_3_2(w);
                break;
            case 9:
                result = curva_ala_3_3(w);
                break;
            case 10:
                result = curva_ala_4_2(w);
                break;
            case 11:
                result = curva_ala_4_3(w);
                break;
            case 12:
                result = curva_ala_4_4(w);
                break;
            case 13:
                result = curva_ala_5_4(w);
                break;
            case 14:
                result = curva_ala_5_5(w);
                break;
            case 15:
                result = curva_ala_5_6(w);
                break;
            case 16:
                result = curva_ala_6_2(w);
                break;
            case 17:
                result = curva_ala_6_3(w);
                break;
            case 18:
                result = curva_ala_6_4(w);
                break;
            case 19:
                result = curva_ala_7_4(w);
                break;
            case 20:
                result = curva_ala_7_5(w);
                break;
            case 21:
                result = curva_ala_7_6(w);
                break;
            default:
                result.xp =0.0;
                result.yp = 0.0;
                result.dxp = 0.0;
                result.dyp = 0.0;     
        }
        return result;
    }

        

    float normalizeAngle(float angle) {
        const double twoPi = 2.0 * M_PI;
        // Normalizar el ángulo para que esté entre 0 y 2pi
        while (angle < 0.0)
            angle += twoPi;
        while (angle >= twoPi)
            angle -= twoPi;
        return angle;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "delta_SGLOS"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    delta_SGLOS = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "k_u_tar"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    k_u_tar = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "taud"){
                if(param.as_double() >= 0.0 and param.as_double() < 500.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    taud = param.as_double();
                    a=(taud*Ts)/(taud*Ts+Ts);
                    b=1/(taud*Ts+Ts);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "path_d"){
                if(param.as_int() >= 0 and param.as_int() <= 21){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    path_d = param.as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-12");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "flag"){
                RCLCPP_INFO(this->get_logger(), "changed param value");
                flag = param.as_bool();
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    bool armed = false, armed_act=false, flag;
    float u_hat = 0, psi_hat = 0, r_hat = 0, v_hat = 0, x_hat = 0, y_hat = 0, u_d = 0, w=0.0, psi_ant;
    int count=0, laps=0;
    //------Params-------//
    float Ts;  
    /*Parámetros del controlador SGLOS*/
    float delta_SGLOS; /*Ganancia delta SGLOS*/
    float k_u_tar; /*Ganancia de la velocidad de surge target*/
    
    float taud; /*Constante tau del filtro derivativo*/
    float a ,b; /*Constantes del filtro derivativo*/

    int path_d; /*Variable para elegir path*/

    std::vector<float> memory_psi; // Memoria para filtro derivativo psi

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_llc;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_error;
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
    auto node = std::make_shared<WangMlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}