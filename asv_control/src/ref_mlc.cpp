#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller

using std::placeholders::_1;


class RefMlcNode : public rclcpp::Node
{
public:
    RefMlcNode() : Node("ref_mlc") 
    {
        subscriber_rc_in = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",1,
                std::bind(&RefMlcNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&RefMlcNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_ref = this-> create_publisher<std_msgs::msg::Float64>("/control/reference_mlc",1);
    	RCLCPP_INFO(this->get_logger(), "Reference Vel Mid Level Node has been started.");
    }

private:

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
    {
        if(armed==true){
            float u_ref=0.0;
            auto ref = std_msgs::msg::Float64();
            u_ref = normalizePwm(msg->channels[2]);
            ref.data = u_ref; 
            publisher_ref->publish(ref);
        }
    }

    float normalizePwm(uint16_t PWM){
        float ref_vel;
        if(PWM <= 1550 && PWM >= 1450){
            PWM = 1500;
            ref_vel = 0.0;
        }else if(PWM > 1550){
            PWM = PWM - 50;
            ref_vel = ((PWM*0.002857142)-4.285714286);
        }else if(PWM < 1450){
            PWM = PWM + 50;
            ref_vel = ((PWM*0.002857142)-4.285714286);
        }
                
        if(ref_vel>1.0){
            ref_vel = 1.0;
        }else if (ref_vel<=-1.0)
        {
            ref_vel = -1.0;
        }

        if(ref_vel>=0.9){
            ref_vel = 1.0;
        }else if(ref_vel>=0.7 && ref_vel<0.9){
            ref_vel = 0.75;
        }else if(ref_vel>=0.4 && ref_vel<0.7){
            ref_vel = 0.5;
        }else if(ref_vel>=0.1 && ref_vel<0.4){
            ref_vel = 0.3;
        }
        
        return ref_vel;
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        //if(msg->armed == true && msg->armed!=armed){
        //    RCLCPP_INFO(this->get_logger(), "Initial psi= %f", psi_act);
        //}
        armed= msg->armed;
    }

    bool armed = false;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_ref;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RefMlcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}