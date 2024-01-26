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
            if(msg->channels[2] < 1550 && msg->channels[2] > 1450){
                msg->channels[2] = 1500;
            }
            u_ref = 0.5*((msg->channels[2]*0.0025)-3.75);
            auto ref = std_msgs::msg::Float64();
            if(u_ref>1.0){
                u_ref = 1.0;
            }else if (u_ref<=-1.0)
            {
                u_ref = -1.0;
            }
            ref.data = u_ref; 
            publisher_ref->publish(ref);
        }
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