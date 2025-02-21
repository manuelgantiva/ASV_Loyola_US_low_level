
// Matrix<float, 4, 1> q_p_r = {0.4*w + 20*sin(0.02*w), 0.4*w, 0.4 + 0.4*cos(0.02*w), 0.4};

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <Eigen/Dense>
#include <cmath>
#include <thread>
#include "asv_library/curvas_loyola.h"

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

// Declaración de contantes

class ModWangMlcNode : public rclcpp::Node
{
public:
    ModWangMlcNode() : Node("mod_wang_mlc")
    {     
        std::string my_string_id; 
        this-> declare_parameter("my_id", "ASV0");
        
        //---------Parámetros del MLC-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("k1", 0.005);
        this-> declare_parameter("k2", 100.0);
        this-> declare_parameter("taud", 15.0); // Taud = #*Ts Est es #
        this-> declare_parameter("u_max", 1.5); // path_d = #Path deseado #

        my_string_id = (this->get_parameter("my_id").as_string());    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        k1 = this->get_parameter("k1").as_double();
        k2 = this->get_parameter("k2").as_double();
        taud = this->get_parameter("taud").as_double();
        u_max = this->get_parameter("u_max").as_double();

        memory_psi.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        // params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ModWangMlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_string_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&ModWangMlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_hlc_ = this-> create_subscription<asv_interfaces::msg::StateObserver>("/" + my_string_id + "/control/output_hlc",
            rclcpp::SensorDataQoS(), std::bind(&ModWangMlcNode::callbackHLCReference,this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_string_id + "/mavros/state",
            1,std::bind(&ModWangMlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        publisher_llc = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_string_id + "/control/reference_llc",1);
        publisher_error = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_string_id + "/control/error_mlc",1);

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&ModWangMlcNode::calculateMidLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Mid Level Controller Wang Node in %s has been started.", my_string_id.c_str());
    	
    }

private:
    void calculateMidLevelController()
    {
        if(armed==false){
            memory_psi.assign(4, 0.0);
            count=0;
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
                float xe, ye;
                float psip_i;

                Matrix<float, 2, 1> p_i, q_i;
                p_i.setZero();
                q_i.setZero();

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    x_hat_i = x_hat;
                    y_hat_i = y_hat;
                    v_hat_i=v_hat;
                    psi_hat_i=psi_hat;
                    
                    q_i = {q_x_i, q_y_i};
                    p_i= {p_x_i, p_y_i};
                }
                psip_i = atan2(p_i(1, 0), p_i(0, 0));

                psi_hat_i = normalizeAngle(psi_hat_i);

                xe = (x_hat_i - q_i(0, 0))*cos(psip_i) + (y_hat_i - q_i(1, 0))*sin(psip_i);
                ye = -1*(x_hat_i - q_i(0, 0))*sin(psip_i) + (y_hat_i - q_i(1, 0))*cos(psip_i);

                msg_e.x = xe;
                msg_e.y = ye;
                // msg_e.z = w;

                float k1_i = k1, k2_i = k2;
                float u_ref = p_i.norm()  + k1_i * std::abs(ye);
                float b_ref = 0.0;
                b_ref = atan2(v_hat_i, u_ref);
                
                float psi_ref = psip_i - b_ref - atan2(ye, k2_i*std::abs(xe));
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

                // TODO: Implementar saturación de velocidad angular
                float r_ref_max = 0.6;
                if(r_ref > r_ref_max){
                    r_ref = r_ref_max;
                }else if(r_ref < -r_ref_max){
                    r_ref = -r_ref_max;
                }

                if(u_ref > u_max){
                    u_ref = u_max;
                }
                
                msg.x = u_ref;
                msg.y = r_ref;
                msg.z = psi_ref;

                publisher_llc->publish(msg);
                publisher_error->publish(msg_e);
                armed_act = armed;
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

    void callbackHLCReference(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            q_x_i = msg->point.x;
            q_y_i = msg->point.y;
            p_x_i = msg->velocity.x;
            p_y_i = msg->velocity.y;
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

    float normalizeAngle(float angle) {
        const double twoPi = 2.0 * M_PI;
        // Normalizar el ángulo para que esté entre 0 y 2pi
        while (angle < 0.0)
            angle += twoPi;
        while (angle >= twoPi)
            angle -= twoPi;
        return angle;
    }

    // rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
    //     rcl_interfaces::msg::SetParametersResult result;
    //     for (const auto &param: params){
    //         if (param.get_name() == "delta_SGLOS"){
    //             if(param.as_double() >= 0.0 and param.as_double() < 100.0){
    //                 RCLCPP_INFO(this->get_logger(), "changed param value");
    //                 delta_SGLOS = param.as_double();
    //             }else{
    //                 RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
    //                 result.successful = false;
    //                 result.reason = "Value out of range";
    //                 return result;
    //             }
    //         }
    //         if (param.get_name() == "k_u_tar"){
    //             if(param.as_double() >= 0.0 and param.as_double() < 100.0){
    //                 RCLCPP_INFO(this->get_logger(), "changed param value");
    //                 k_u_tar = param.as_double();
    //             }else{
    //                 RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
    //                 result.successful = false;
    //                 result.reason = "Value out of range";
    //                 return result;
    //             }
    //         }
    //         if (param.get_name() == "taud"){
    //             if(param.as_double() >= 0.0 and param.as_double() < 500.0){
    //                 RCLCPP_INFO(this->get_logger(), "changed param value");
    //                 taud = param.as_double();
    //                 a=(taud*Ts)/(taud*Ts+Ts);
    //                 b=1/(taud*Ts+Ts);
    //             }else{
    //                 RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
    //                 result.successful = false;
    //                 result.reason = "Value out of range";
    //                 return result;
    //             }
    //         }
    //         if (param.get_name() == "path_d"){
    //             if(param.as_int() >= 0 and param.as_int() <= 12){
    //                 RCLCPP_INFO(this->get_logger(), "changed param value");
    //                 path_d = param.as_int();
    //             }else{
    //                 RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-12");
    //                 result.successful = false;
    //                 result.reason = "Value out of range";
    //                 return result;
    //             }
    //         }
    //         if (param.get_name() == "flag"){
    //             RCLCPP_INFO(this->get_logger(), "changed param value");
    //             flag = param.as_bool();
    //         }
    //     }
    //     result.successful = true;
    //     result.reason = "Success";
    //     return result;
    // }

    bool armed = false, armed_act=false;
    float u_hat = 0, psi_hat = 0, r_hat = 0, v_hat = 0, x_hat = 0, y_hat = 0, psi_ant=0;
    float p_x_i=0, p_y_i=0, q_x_i=0, q_y_i=0;
    int count=0, laps=0;
    //------Params-------//
    float Ts;  
    float k1=0.005, k2=100; /*Ganancia de la velocidades u, psi, de referencia*/
    float u_max; /*velocidad maxima de referencia*/
    
    float taud; /*Constante tau del filtro derivativo*/
    float a ,b; /*Constantes del filtro derivativo*/

    std::vector<float> memory_psi; // Memoria para filtro derivativo psi

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_hlc_;
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
    auto node = std::make_shared<ModWangMlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}