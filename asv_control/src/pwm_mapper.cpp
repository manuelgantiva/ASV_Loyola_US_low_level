#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include <asv_interfaces/msg/pwm_values.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class PwmMapperNode : public rclcpp::Node 
{
public:
    PwmMapperNode() : Node("pwm_mapper")
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        my_id = (this->get_parameter("my_id").as_string());
        t_left=1500;
        t_right=1500;
        publisher_ = this-> create_publisher<mavros_msgs::msg::OverrideRCIn>("/" + my_id + "/mavros/rc/override",10);
        /*timer_ = this -> create_wall_timer(std::chrono::milliseconds(100),
                                          std::bind(&PwmMapperNode::publishOverridePwm, this));*/
        server_ = this-> create_service<example_interfaces::srv::SetBool>(
                "/" + my_id + "/control/on_off_pwm", std::bind(&PwmMapperNode::callbackOnOffPwm, this, _1, _2));
        subscriber_ = this-> create_subscription<asv_interfaces::msg::PwmValues>("/" + my_id + "/control/pwm_values",10,
                std::bind(&PwmMapperNode::callbackPwmValues, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Pwm Mapper Node in %s has been started.", my_id.c_str());

    }

private:
    void callbackPwmValues(const asv_interfaces::msg::PwmValues::SharedPtr msg)
    {
        /*t_left=msg->t_left;
        t_right=msg->t_righ;*/
        auto msg_rc = mavros_msgs::msg::OverrideRCIn();
        if(this->on_off_pwm){
            msg_rc.channels =  std::array<uint16_t, 18>{0, 0, 0, 0, 0, 0, 0, 0, 0, msg->t_left, msg->t_righ, 0, 0, 0, 0, 0, 0, 0};
        }else{
            msg_rc.channels =  std::array<uint16_t, 18>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        }
        publisher_->publish(msg_rc);
    }

    void callbackOnOffPwm(const example_interfaces::srv::SetBool::Request::SharedPtr request,
                            const example_interfaces::srv::SetBool::Response::SharedPtr response)
    {
        // true the control pwm is override
        if(request->data == true){
            this->on_off_pwm=true;
        }
        // false the control pwm is ardupilot
        if(request->data == false){
            this->on_off_pwm=false;
        }
        response->success = true;
        response->message = "ok";
    }

    /*void publishOverridePwm()
    {
        
    }*/

    uint16_t t_left, t_right;
    bool on_off_pwm=false;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    rclcpp::Subscription<asv_interfaces::msg::PwmValues>::SharedPtr subscriber_;
    std::vector<std::thread> threads_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PwmMapperNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}