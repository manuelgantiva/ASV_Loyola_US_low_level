#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "mavros_msgs/srv/param_set_v2.hpp"         //Interface srv set param mavros 
#include "mavros_msgs/srv/command_home.hpp"         //Interface srv set home mavros 
#include "mavros_msgs/srv/command_bool.hpp"         //Interface srv set armed mavros
#include "mavros_msgs/srv/set_mode.hpp"             //Interface srv set mode guided mavros
#include "example_interfaces/srv/set_bool.hpp"      //Interface srv on_off PWM
#include "asv_interfaces/srv/set_llc.hpp"           //Interface srv set low level controller enable 
#include "asv_interfaces/srv/set_obs.hpp"           //Interface srv set State Observer enable 

#include <cmath>

using std::placeholders::_1;

const uint8_t MANUAL = 1;
const uint8_t AUTO = 2;
const uint8_t ROS = 3;

class RcHandlerNode : public rclcpp::Node
{
public:
    RcHandlerNode() : Node("rc_handler") 
    {
        this->client_set_mode_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        this->client_set_llc_ = this->create_client<asv_interfaces::srv::SetLlc>("/control/set_llc");
        this->client_set_obs_ = this->create_client<asv_interfaces::srv::SetObs>("/control/set_obs");
        this->client_set_param = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");
        this->client_enable_pwm = this->create_client<example_interfaces::srv::SetBool>("/control/on_off_pwm");
        this->client_set_home = this->create_client<mavros_msgs::srv::CommandHome>("/mavros/cmd/set_home");
        this->client_arming = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
        subscriber_ = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",10,
                std::bind(&RcHandlerNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_2 = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",10,
                std::bind(&RcHandlerNode::callbackMavrosState, this, std::placeholders::_1));
    	RCLCPP_INFO(this->get_logger(), "Rc Handler Node has been started.");
    }

private:

    void callParamSetServoOut(std::string Servo, int Value)
    {
        while (!client_set_param->wait_for_service(std::chrono::seconds(1)))
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

        client_set_param->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseSetServoOut, this, _1));
    }

    void callbackResponseSetServoOut(rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFutureWithRequest future){
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

    void callSetLowController(uint8_t llc_enable)
    {
        while (!client_set_llc_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server set_llc to be up...");
        }

        auto request = std::make_shared<asv_interfaces::srv::SetLlc::Request>();
        request->llc_mode = llc_enable;

        client_set_llc_->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseSetLlc, this, _1));
    }

    void callbackResponseSetLlc(rclcpp::Client<asv_interfaces::srv::SetLlc>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"set mode succes = %d",  int(response.second->success));                  
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callSetStateObserver(uint8_t obs_enable)
    {
        while (!client_set_obs_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server set_obs to be up...");
        }

        auto request = std::make_shared<asv_interfaces::srv::SetObs::Request>();
        request->eso_mode = obs_enable;

        client_set_obs_->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseSetObs, this, _1));
    }

    void callbackResponseSetObs(rclcpp::Client<asv_interfaces::srv::SetObs>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"set mode succes = %d",  int(response.second->success));                  
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callSetHomeMavros()
    {
        while (!client_set_home->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server set_home to be up...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
        request->current_gps = false;
        request->yaw = 0.0;
        request->latitude= 37.3076703;
        request->longitude= -5.9402279;
        request->altitude = 89.95863749032833
;

        client_set_home->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseSetHome, this, _1));
    }

    void callbackResponseSetHome(rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"succes: %d and new value is %d",  
                        response.second->success, int(response.second->result));                  
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callArming(bool armed)
    {
        while (!client_arming->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server arming to be up...");
        }

        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = armed;
        
        client_arming->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseArming, this, _1));
    }

    void callbackResponseArming(rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(),"succes: %d and new value is %d",  
                        response.second->success, int(response.second->result));                  
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callSetModeMavros(std::string new_mode)
    {
        while (!client_set_mode_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for set mode server to be up...");
        }

        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->base_mode = 0;
        request->custom_mode = new_mode;

        client_set_mode_->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseSetMode, this, _1));
    }
    
    void callbackResponseSetMode(rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFutureWithRequest future){
        try
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "set mode succes = %d",  int(response.second->mode_sent));
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
        }
    }

    void callOnOffPwm(bool mode)      // True on and False off PWM Override
    {
        while (!client_enable_pwm->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server on_off_pwm to be up...");
        }

        auto request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        request->data = mode;

        client_enable_pwm->async_send_request(request,std::bind(&RcHandlerNode::callbackResponseEnablePwm, this, _1));
    }

    void callbackResponseEnablePwm(rclcpp::Client<example_interfaces::srv::SetBool>::SharedFutureWithRequest future){
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
        auto mode_act = uint8_t();
        sel_obs= msg->channels[3];
        sel_con= msg->channels[1];
        if(msg->channels[5]>1700){
            mode_act = MANUAL;
        }else if (msg->channels[5]<1300)
        {
            mode_act = AUTO;
        }else{
            mode_act = ROS;
        }

        if(mode_act!=mode){
            switch(mode_act) {
                case MANUAL:
                    RCLCPP_INFO(this->get_logger(), "Manual mode");
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this,
                                        "SERVO1_FUNCTION", 74)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                        "SERVO3_FUNCTION", 73)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callOnOffPwm, this, false)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetModeMavros, this, "MANUAL")));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callArming, this, false)));
                    break;
                case AUTO:
                    RCLCPP_INFO(this->get_logger(), "Ardupilot control mode");
                    
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this,
                                        "SERVO1_FUNCTION", 74)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                        "SERVO3_FUNCTION", 73)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callOnOffPwm, this, false)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callArming, this, false)));
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetModeMavros, this, "MANUAL")));
                    break;
                case ROS:
                    RCLCPP_INFO(this->get_logger(), "ROS2 control mode");
                    threads_.push_back(std::thread(std::bind(&RcHandlerNode::callArming, this, true)));
                    if(sel_con<1400){
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetModeMavros, this, "GUIDED")));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetLowController, this, 
                                            asv_interfaces::srv::SetLlc::Request::LLC_APM)));
                    }else if(sel_con>1600){
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                            "SERVO1_FUNCTION", 61)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                            "SERVO3_FUNCTION", 60)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callOnOffPwm, this, true)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetLowController, this, 
                                            asv_interfaces::srv::SetLlc::Request::LLC_IFAC)));
                    }else{
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                            "SERVO1_FUNCTION", 61)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callParamSetServoOut, this, 
                                            "SERVO3_FUNCTION", 60)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callOnOffPwm, this, true)));
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetLowController, this, 
                                            asv_interfaces::srv::SetLlc::Request::LLC_MPC)));
                    }
                    if(sel_obs<1400){
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetStateObserver, this, 
                                            asv_interfaces::srv::SetObs::Request::ESO_ZONO)));
                    }else if(sel_obs>1600){
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetStateObserver, this, 
                                            asv_interfaces::srv::SetObs::Request::ESO_LIU)));
                    }else{
                        threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetStateObserver, this, 
                                            asv_interfaces::srv::SetObs::Request::ESO_GUILLE)));
                    }
                    break;
                default:
                    RCLCPP_INFO(this->get_logger(), "Unknown mode");
                    break;
            }
        this->mode = mode_act;
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if(msg->armed == true && msg->armed!=armed){
            RCLCPP_INFO(this->get_logger(), "Set Home");
            threads_.push_back(std::thread(std::bind(&RcHandlerNode::callSetHomeMavros, this)));
        }
        armed= msg->armed;
        RCLCPP_INFO(this->get_logger(), "mode: %s, manual input: %d, armed: %d", msg->mode.c_str(),
                        msg->manual_input, msg->armed);
    }

    bool armed = false;
    uint16_t sel_obs, sel_con;
    uint8_t mode = 1;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_2;
    rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr client_set_param;
    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr client_set_home;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_arming;
    rclcpp::Client<example_interfaces::srv::SetBool>::SharedPtr client_enable_pwm;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_set_mode_;
    rclcpp::Client<asv_interfaces::srv::SetLlc>::SharedPtr client_set_llc_;
    rclcpp::Client<asv_interfaces::srv::SetObs>::SharedPtr client_set_obs_;
    std::vector<std::thread> threads_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RcHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}