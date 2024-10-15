#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface compass initial psi

#include <cmath>

#define PI 3.141592

using std::placeholders::_1;


class RefLlcNode : public rclcpp::Node
{
public:
    RefLlcNode() : Node("ref_llc") 
    {
        this-> declare_parameter("reference_mode", false);
        reference_mode_ = this->get_parameter("reference_mode").as_bool();

        subscriber_compass = this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",rclcpp::SensorDataQoS(),
                std::bind(&RefLlcNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_rc_in = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",10,
                std::bind(&RefLlcNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&RefLlcNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_ref = this-> create_publisher<geometry_msgs::msg::Vector3>("/control/reference_llc",1);
    	RCLCPP_INFO(this->get_logger(), "Reference Vel Node has been started.");
    }

private:

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
    {
        if(armed==true){
            float u_ref=0.0;
            float r_ref=0.0;

            auto ref = geometry_msgs::msg::Vector3();

            u_ref = normalizePwmSurge(msg->channels[2]);
            r_ref = normalizePwmYaw(msg->channels[0]);

            if(reference_mode_ == true){
                ref.x = u_ref;
                ref.y = r_ref;
            }else{
                float periodo = 10.0; // Período en segundos
                u_ref = u_ref/2;
                r_ref = r_ref/2;
                ref.x = u_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+u_ref;
                ref.y = r_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+r_ref;
                counter_++;
            }
                        
            psi_act = psi_act+(ref.y*0.1);
            ref.z = psi_act;
            publisher_ref->publish(ref);
        }
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
        ref_vel = round(ref_vel / 0.2) * 0.2;
        ref_vel = ref_vel * 1.5;
        return ref_vel;
    }

    float normalizePwmYaw(uint16_t PWM){
        float ref_vel;
        if(PWM <= 1550 && PWM >= 1450){
            PWM = 1500;
            ref_vel = 0.0;
        }else if(PWM > 1550){
            PWM = PWM - 50;
        }else if(PWM < 1450){
            PWM = PWM + 50;
        }
        PWM = PWM - 1150;
        ref_vel = ((PWM*3.0/1750.0)-0.6);     
        if(ref_vel>0.6){
            ref_vel = 0.6;
        }else if (ref_vel<=-0.6)
        {
            ref_vel = -0.6;
        }

        // Cuantizar 16 pasos
        ref_vel = round(ref_vel / 0.08) * 0.08;
        //ref_vel = ref_vel * 3.0;
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
            RCLCPP_INFO(this->get_logger(), "Initial psi= %f", psi_act);
        }
        armed= msg->armed;
    }

    bool armed = false;
    float init_psi = 0.0, psi_act = 0.0;
    bool reference_mode_;
    int counter_=0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_ref;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RefLlcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
