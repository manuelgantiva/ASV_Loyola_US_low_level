#include "rclcpp/rclcpp.hpp"
#include "asv_interfaces/srv/set_llc.hpp"           //Interface srv set low level controller enable 
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros

using std::placeholders::_1;
using std::placeholders::_2;

class MuxLlcNode : public rclcpp::Node 
{
public:
    MuxLlcNode() : Node("mux_llc") 
    {
        subscriber_pwm_mpc_ = this-> create_subscription<asv_interfaces::msg::PwmValues>("/control/pwm_value_mpc",
            rclcpp::SensorDataQoS(), std::bind(&MuxLlcNode::callbackPwmValueMpc, this, std::placeholders::_1));
        subscriber_pwm_ifac_ = this-> create_subscription<asv_interfaces::msg::PwmValues>("/control/pwm_value_ifac",
            rclcpp::SensorDataQoS(), std::bind(&MuxLlcNode::callbackPwmValueIfac, this, std::placeholders::_1));
        publisher_pwm_ = this-> create_publisher<asv_interfaces::msg::PwmValues>("/control/pwm_values", 1);
        server_set_llc_ = this-> create_service<asv_interfaces::srv::SetLlc>(
                "/control/set_llc", std::bind(&MuxLlcNode::callbackSetLowLevelControl, this, _1, _2));  
        subscriber_state_mavros_ = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&MuxLlcNode::callbackMavrosState, this, std::placeholders::_1));   
        RCLCPP_INFO(this->get_logger(), "Mux Low Level Controler Node has been started.");
    }

private:
    void callbackPwmValueMpc(const asv_interfaces::msg::PwmValues::SharedPtr msg)
    {
        if(mpc_enable){
            auto msg_p = asv_interfaces::msg::PwmValues();
            msg_p.t_left = msg->t_left;
            msg_p.t_righ = msg->t_righ;
            publisher_pwm_ ->publish(msg_p);
        }
    }

    void callbackPwmValueIfac(const asv_interfaces::msg::PwmValues::SharedPtr msg)
    {
        if(ifac_enable){
            auto msg_p = asv_interfaces::msg::PwmValues();
            msg_p.t_left = msg->t_left;
            msg_p.t_righ = msg->t_righ;
            publisher_pwm_ ->publish(msg_p);
        }
    }

    void callbackSetLowLevelControl(const asv_interfaces::srv::SetLlc::Request::SharedPtr request,
                            const asv_interfaces::srv::SetLlc::Response::SharedPtr response)
    {
        switch (request->llc_mode) {
        case asv_interfaces::srv::SetLlc::Request::LLC_APM:
            RCLCPP_INFO(this-> get_logger(), "APM controller enable");
            apm_enable = true;
            mpc_enable = false;
            ifac_enable = false;
            response->success= true;
            break;
        case asv_interfaces::srv::SetLlc::Request::LLC_MPC:
            RCLCPP_INFO(this-> get_logger(), "MPC controller enable");
            apm_enable = false;
            mpc_enable = true;
            ifac_enable = false;
            response->success= true;
            break;
        case asv_interfaces::srv::SetLlc::Request::LLC_IFAC:
            RCLCPP_INFO(this-> get_logger(), "IFAC controller enable");
            apm_enable = false;
            mpc_enable = false;
            ifac_enable = true;
            response->success= true;
            break;
        default:
            response->success= false;
            RCLCPP_INFO(this-> get_logger(), "request out of range");
            break;
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if(msg->armed == false && msg->armed!=armed){
            apm_enable = false;
            mpc_enable = false;
            ifac_enable = false;
        }
        armed= msg->armed;
    }

    bool armed = false;

    std::vector<std::thread> threads_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state_mavros_;
    rclcpp::Service<asv_interfaces::srv::SetLlc>::SharedPtr server_set_llc_;
    rclcpp::Subscription<asv_interfaces::msg::PwmValues>::SharedPtr subscriber_pwm_mpc_;
    rclcpp::Subscription<asv_interfaces::msg::PwmValues>::SharedPtr subscriber_pwm_ifac_;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm_;
    bool apm_enable = false, mpc_enable = false, ifac_enable = false;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MuxLlcNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}