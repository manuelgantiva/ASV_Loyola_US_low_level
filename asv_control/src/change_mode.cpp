#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/param_set_v2.hpp"
#include "example_interfaces/srv/set_bool.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ChangeModeNode : public rclcpp::Node
{
public:
    ChangeModeNode() : Node("change_mode") 
    {
        this->client_ = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");
        server_ = this-> create_service<example_interfaces::srv::SetBool>(
                "/control/set_mode_pwm", std::bind(&ChangeModeNode::callbackSetModePwm, this, _1, _2));
    	RCLCPP_INFO(this->get_logger(), "Change Mode Node has been started.");
    }

private:
    void callbackSetModePwm(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        // true the control pwm is override
        if(request->data == true){
            threads_.push_back(std::thread(std::bind(&ChangeModeNode::callParamSetServoOut, this, "SERVO1_FUNCTION", 54)));
            threads_.push_back(std::thread(std::bind(&ChangeModeNode::callParamSetServoOut, this, "SERVO3_FUNCTION", 53)));
        }
        // false the control pwm is ardupilot
        if(request->data == false){
            threads_.push_back(std::thread(std::bind(&ChangeModeNode::callParamSetServoOut, this, "SERVO1_FUNCTION", 74)));
            threads_.push_back(std::thread(std::bind(&ChangeModeNode::callParamSetServoOut, this, "SERVO3_FUNCTION", 73)));
        }
        response->success = true;
        response->message = "ok";
    }

    void callParamSetServoOut(std::string Servo, int Value)
    {
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server set_param to be up...");
        }

        auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();
        request->force_set = false;
        request->param_id = Servo;
        request->value.type = 2;
        request->value.bool_value = false;
        request->value.integer_value = Value;
        request->value.double_value = 0.0;

        client_->async_send_request(request,std::bind(&ChangeModeNode::response_received_callback, this, _1));
    }
    
    void response_received_callback(rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"succes: %d and new value is %d",  
                        response.second->success, int(response.second->value.integer_value));                  
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr client_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    std::vector<std::thread> threads_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ChangeModeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}