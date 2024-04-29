#include "rclcpp/rclcpp.hpp"
#include "asv_interfaces/srv/set_obs.hpp"               //Interface srv set state observer enable 
#include "asv_interfaces/msg/state_observer.hpp"        //Interface result state observer
#include "mavros_msgs/msg/state.hpp"                    //Interface state mavros
#include "geometry_msgs/msg/twist_stamped.hpp"       //Interface velocity APM

using std::placeholders::_1;
using std::placeholders::_2;

class MuxObsNode : public rclcpp::Node 
{
public:
    MuxObsNode() : Node("mux_obs") 
    {
        server_set_obs_ = this-> create_service<asv_interfaces::srv::SetObs>(
                "/control/set_obs", std::bind(&MuxObsNode::callbackSetStateObserver, this, _1, _2)); 
        subscriber_state_mavros_ = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&MuxObsNode::callbackMavrosState, this, std::placeholders::_1));   
        subscriber_state_guille_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer_guille", rclcpp::SensorDataQoS(), 
            std::bind(&MuxObsNode::callbackStatesGuille, this, std::placeholders::_1));
        subscriber_state_liu_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer_liu", rclcpp::SensorDataQoS(), 
            std::bind(&MuxObsNode::callbackStatesLiu, this, std::placeholders::_1));
        publisher_state_ = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer",
                rclcpp::SensorDataQoS());

        RCLCPP_INFO(this->get_logger(), "Mux state observer Node has been started.");
    }

private:
    void callbackStatesGuille(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        if(guille_enable){
            auto msg_p = asv_interfaces::msg::StateObserver();
            msg_p.header = msg->header;
            msg_p.point = msg->point;
            msg_p.velocity = msg->velocity;
            msg_p.disturbances = msg->disturbances;
            publisher_state_ ->publish(msg_p);
        }
    }

    void callbackStatesLiu(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        if(liu_enable){
            auto msg_p = asv_interfaces::msg::StateObserver();
            msg_p.header = msg->header;
            msg_p.point = msg->point;
            msg_p.velocity = msg->velocity;
            msg_p.disturbances = msg->disturbances;
            publisher_state_ ->publish(msg_p);
        }
    }

    void callbackSetStateObserver(const asv_interfaces::srv::SetObs::Request::SharedPtr request,
                            const asv_interfaces::srv::SetObs::Response::SharedPtr response)
    {
        switch (request->eso_mode) {
        case asv_interfaces::srv::SetObs::Request::ESO_GUILLE:
            RCLCPP_INFO(this-> get_logger(), "Guille ESO enable");
            guille_enable = true;
            liu_enable = false;
            zono_enable = false;
            response->success= true;
            break;
        case asv_interfaces::srv::SetObs::Request::ESO_LIU:
            RCLCPP_INFO(this-> get_logger(), "Liu ESO enable");
            guille_enable = false;
            liu_enable = true;
            zono_enable = false;
            response->success= true;
            break;
        case asv_interfaces::srv::SetObs::Request::ESO_ZONO:
            RCLCPP_INFO(this-> get_logger(), "Zonotopos ESO enable");
            guille_enable = false;
            liu_enable = false;
            zono_enable = true;
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
            guille_enable = false;
            liu_enable = false;
            zono_enable = false;
        }
        armed= msg->armed;
    }

    bool armed = false;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state_mavros_;
    rclcpp::Service<asv_interfaces::srv::SetObs>::SharedPtr server_set_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_guille_;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_liu_;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_;

    bool guille_enable = false, liu_enable = false, zono_enable = false;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MuxObsNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}