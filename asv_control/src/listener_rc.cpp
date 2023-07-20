#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "example_interfaces/srv/set_bool.hpp"

#include <cmath>

using std::placeholders::_1;


class ListenerRcNode : public rclcpp::Node
{
public:
    ListenerRcNode() : Node("listener_rc") 
    {
        this->client_ = this->create_client<example_interfaces::srv::SetBool>("/control/set_mode_pwm");
        this->client_2 = this->create_client<example_interfaces::srv::SetBool>("/control/on_off_pwm");
        subscriber_ = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",10,
                std::bind(&ListenerRcNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_2 = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",10,
                std::bind(&ListenerRcNode::callbackMavrosState, this, std::placeholders::_1));
    	RCLCPP_INFO(this->get_logger(), "Listener Rc Node has been started.");
    }

private:
    void callSetModePwm(bool mode)      // True override and False Throttle
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server set_mode_pwm to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = mode;

        client_->async_send_request(request,std::bind(&ListenerRcNode::response_received_callback, this, _1));
    }

    void callOnOffPwm(bool mode)      // True on and False off PWM Override
    {
        while (!client_2->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server on_off_pwm to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = mode;

        client_2->async_send_request(request,std::bind(&ListenerRcNode::response_received_callback, this, _1));
    }

    void response_received_callback(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "succes: %d and msg is %s",
                    int(response.second->success) , response.second->message.c_str());
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
    {
        if(abs(msg->channels[5]-channel6_pre)>10){
            if(msg->channels[5]>1700){
                RCLCPP_INFO(this->get_logger(), "Manual mode");
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callSetModePwm, this, false)));
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callOnOffPwm, this, false)));
            }else if (msg->channels[5]<1300)
            {
                RCLCPP_INFO(this->get_logger(), "Ardupilot control mode");
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callSetModePwm, this, false)));
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callOnOffPwm, this, false)));
            }else{
                RCLCPP_INFO(this->get_logger(), "ROS2 control mode");
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callSetModePwm, this, true)));
                threads_.push_back(std::thread(std::bind(&ListenerRcNode::callOnOffPwm, this, true)));
            }
        }
        channel6_pre= msg->channels[5];
        RCLCPP_INFO(this->get_logger(), "%d", int(channel6_pre));
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "mode: %s, manual input: %d, armed: %d", msg->mode.c_str(),
                        msg->manual_input, msg->armed);
    }

    uint16_t channel6_pre = 1900;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_2;
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_;
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_2;
    std::vector<std::thread> threads_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ListenerRcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}