#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "geometry_msgs/msg/twist.hpp"              //Interface cmd vel ardupilot

#include <cmath>
#include <thread>

class ApmLlcNode : public rclcpp::Node
{
public:
    ApmLlcNode() : Node("apm_llc")
    {     
        
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);

        Ts = this->get_parameter("Ts").as_double();

        subscriber_references_ = this-> create_subscription<geometry_msgs::msg::Vector3>(
            "/control/reference_llc", 1, std::bind(&ApmLlcNode::callbackVelReference,
            this, std::placeholders::_1));
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ApmLlcNode::callbackStateData, this, std::placeholders::_1));
        publisher_cmd_vel_ = this-> create_publisher<geometry_msgs::msg::Twist>(
                "/mavros/setpoint_velocity/cmd_vel_unstamped", 1);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller APM Node has been started.");
    	
    }

private:
    
    void callbackVelReference(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        auto cmd_vel = geometry_msgs::msg::Twist();
        cmd_vel.linear.x = msg->x;
        cmd_vel.angular.z = msg->y;
        publisher_cmd_vel_ ->publish(cmd_vel);
    }

    
    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    bool armed = false;

    //------Params-------//
    float Ts;  

    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ApmLlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}