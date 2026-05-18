#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface compass initial psi
#include "asv_interfaces/msg/reference_llc.hpp"

#include <cmath>

#define PI 3.141592

using std::placeholders::_1;


class RefLlcNode : public rclcpp::Node
{
public:
    RefLlcNode() : Node("ref_llc") 
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
       
        this-> declare_parameter("reference_mode", false);

        my_id = (this->get_parameter("my_id").as_string());
        reference_mode_ = this->get_parameter("reference_mode").as_bool();

        // Inicialización a cero (opcional, normalmente ROS msgs ya inicializan en cero)
        for (auto &v : history) {
            v.x = 0.0f;
            v.y = 0.0f;
            v.z = 0.0f;
        }

        subscriber_compass = this-> create_subscription<std_msgs::msg::Float64>("/" + my_id + "/mavros/global_position/compass_hdg",rclcpp::SensorDataQoS(),
                std::bind(&RefLlcNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_rc_in = this-> create_subscription<mavros_msgs::msg::RCIn>("/" + my_id + "/mavros/rc/in",10,
                std::bind(&RefLlcNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&RefLlcNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_ref = this-> create_publisher<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1);
    	RCLCPP_INFO(this->get_logger(), "Reference Vel Node in %s has been started.", my_id.c_str());
    }

private:

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
    {
        if(armed==true){
            float u_ref=0.0;
            float r_ref=0.0;

            u_ref = normalizePwmSurge(msg->channels[2]);
            r_ref = normalizePwmYaw(msg->channels[0]);

            if(reference_mode_ == false){
                float periodo = 10.0; // Período en segundos
                u_ref = u_ref/2;
                r_ref = r_ref/2;
                u_ref = u_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+u_ref;
                r_ref = r_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+r_ref;
                counter_++;
            }

            auto msg_ref = asv_interfaces::msg::ReferenceLlc();

            std::vector<geometry_msgs::msg::Vector3> refs_;
            refs_.reserve(static_cast<size_t>(N_t));

            for (int i = 0; i < 10; ++i) {
                refs_.push_back(history[i]);
            }

            float psi_i = psi_act;
            psi_act = psi_act+(r_ref*0.1);
            for (int i = 10; i < N_t; i++)
            {
                auto msg_i = geometry_msgs::msg::Vector3();
                msg_i.x = u_ref;
                msg_i.y = r_ref;
                msg_i.z = psi_i + 0.1*msg_i.y; //psi_ref;
                psi_i = msg_i.z;
                refs_ .emplace_back(std::move(msg_i));
            }

            msg_ref.references = refs_;

            publisher_ref->publish(msg_ref);

            geometry_msgs::msg::Vector3 current_value;
            current_value.x = u_ref;
            current_value.y = r_ref;
            current_value.z = psi_act; // valor calculado en el paso actual

            updateHistory(current_value);
        }
    }

    void updateHistory(const geometry_msgs::msg::Vector3 &new_value) {
        for (size_t i = 1; i < history.size(); ++i) {
            history[i-1] = history[i];
        }
        // Insertar el nuevo valor en la última posición
        history[history.size() - 1] = new_value;
    }

    float normalizePwmSurge(uint16_t PWM){
        float ref_vel;
        if(PWM <= 1550 && PWM >= 1450){
            PWM = 1500;
            ref_vel = 0.0;
        }else if(PWM > 1550){
            PWM = PWM - 50;
        }else if(PWM < 1450){
            PWM = PWM + 50;
        }
        ref_vel = ((PWM*0.002857142)-4.285714286);      
        if(ref_vel>1.0){
            ref_vel = 1.0;
        }else if (ref_vel<=-1.0)
        {
            ref_vel = -1.0;
        }
        // Cuantizar 6 pasos
        ref_vel = round(ref_vel / 0.1) * 0.1;
        ref_vel = ref_vel * 1.2;
        return ref_vel;
    }

    float normalizePwmYaw(uint16_t PWM){
        float pwm_adj;
        float ref_vel;
        if (PWM >= 1450 && PWM <= 1550) {
            pwm_adj = 1500.0f;
        } else if (PWM > 1550) {
            pwm_adj = static_cast<float>(PWM) - 50.0f;
        } else {
            pwm_adj = static_cast<float>(PWM) + 50.0f;
        }
        // Mapear el rango útil [1150, 1850] a [-0.5, 0.5]
        ref_vel = ((pwm_adj - 1150.0f) / 700.0f) - 0.5f;
        // Saturación
        if (ref_vel > 0.5f) {
            ref_vel = 0.5f;
        } else if (ref_vel < -0.5f) {
            ref_vel = -0.5f;
        }
        // Cuantizar en pasos de 0.1
        ref_vel = std::round(ref_vel / 0.1f) * 0.1f;
        // Saturar otra vez por seguridad
        if (ref_vel > 0.5f) {
            ref_vel = 0.5f;
        } else if (ref_vel < -0.5f) {
            ref_vel = -0.5f;
        }
        return ref_vel;
    }

    void callbackCompassData(const std_msgs::msg::Float64::SharedPtr msg)
    {
        init_psi = (msg->data*PI/180);
        //RCLCPP_INFO(this->get_logger(), "Exec time: %f", init_psi);
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if(msg->armed == true && msg->armed!=armed){
            counter_ = 0;
            psi_act = init_psi;
            for (auto &v : history) {
                v.x = 0.0f;
                v.y = 0.0f;
                v.z = init_psi;
            }
            RCLCPP_INFO(this->get_logger(), "Initial psi= %f", psi_act);
        }
        armed= msg->armed;
    }

    bool armed = false;
    float init_psi = 0.0, psi_act = 0.0, N_t = 30;
    bool reference_mode_;
    int counter_=0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<asv_interfaces::msg::ReferenceLlc>::SharedPtr publisher_ref;

    // Historial fijo de 10 posiciones
    std::array<geometry_msgs::msg::Vector3, 10> history;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RefLlcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}