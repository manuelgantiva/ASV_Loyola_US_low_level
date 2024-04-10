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
        this-> declare_parameter("alfa", 0.2);
        alfa = this->get_parameter("alfa").as_double();

        server_set_obs_ = this-> create_service<asv_interfaces::srv::SetObs>(
                "/control/set_obs", std::bind(&MuxObsNode::callbackSetStateObserver, this, _1, _2)); 
        subscriber_vel_body= this-> create_subscription<geometry_msgs::msg::TwistStamped>(
            "/mavros/local_position/velocity_body", rclcpp::SensorDataQoS(), 
            std::bind(&MuxObsNode::callbackVelocityBodyData, this, std::placeholders::_1)); 
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

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MuxObsNode::param_callback, this, _1));

        Disturbance.x = 0.0;
        Disturbance.y = 0.0;
        Disturbance.z = 0.0;

        Velocity.x = 0.0;
        Velocity.y = 0.0;
        Velocity.z = 0.0;

        FilVel.x = 0.0;
        FilVel.y = 0.0;
        FilVel.z = 0.0;

        FilDis.x = 0.0;
        FilDis.y = 0.0;
        FilDis.z = 0.0;

        RCLCPP_INFO(this->get_logger(), "Mux state observer Node has been started.");
    }

private:
    void callbackStatesGuille(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        if(guille_enable){
            auto msg_p = asv_interfaces::msg::StateObserver();
            msg_p.header = msg->header;
            msg_p.point = msg->point;
            FilVel=sumVectors(multiplyByScalar(msg->velocity, alfa), multiplyByScalar(FilVel, 1.0-alfa));
            msg_p.velocity = FilVel;
            FilDis=sumVectors(multiplyByScalar(msg->disturbances, alfa), multiplyByScalar(FilDis, 1.0-alfa));
            msg_p.disturbances = FilDis;
            publisher_state_ ->publish(msg_p);
        }
    }

    void callbackStatesLiu(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        if(liu_enable){
            auto msg_p = asv_interfaces::msg::StateObserver();
            msg_p.header = msg->header;
            msg_p.point = msg->point;
            msg_p.velocity = Velocity;
            msg_p.disturbances = Disturbance;
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
            // Reset the velocity vector to zero 
            Velocity.x = 0.0;
            Velocity.y = 0.0;
            Velocity.z = 0.0;
            // Reset the Filter velocity vector to zero 
            FilVel.x = 0.0;
            FilVel.y = 0.0;
            FilVel.z = 0.0;
            // Reset the FIlter disturbance vector to zero 
            FilDis.x = 0.0;
            FilDis.y = 0.0;
            FilDis.z = 0.0;
        }
        armed= msg->armed;
    }

    void callbackVelocityBodyData(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        if (armed == true)
        {
            Velocity.x = msg->twist.linear.x;
            Velocity.y = msg->twist.linear.y;
            Velocity.z = -1*msg->twist.angular.z;
        }
    }

    // Function to sum two vectors
    geometry_msgs::msg::Vector3 sumVectors(const geometry_msgs::msg::Vector3& vec1, const geometry_msgs::msg::Vector3& vec2) {
        geometry_msgs::msg::Vector3 result;
        result.x = vec1.x + vec2.x;
        result.y = vec1.y + vec2.y;
        result.z = vec1.z + vec2.z;
        return result;
    }

    // Function to multiply a vector by a scalar
    geometry_msgs::msg::Vector3 multiplyByScalar(const geometry_msgs::msg::Vector3& vec, float scalar) {
        geometry_msgs::msg::Vector3 result;
        result.x = vec.x * scalar;
        result.y = vec.y * scalar;
        result.z = vec.z * scalar;
        return result;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "alfa"){
                if(param.as_double() >= 0.0 and param.as_double() <= 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    alfa = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be 0.0 or 1.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    bool armed = false;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state_mavros_;
    rclcpp::Service<asv_interfaces::srv::SetObs>::SharedPtr server_set_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_guille_;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_liu_;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_vel_body;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    geometry_msgs::msg::Vector3 Velocity , Disturbance, FilVel, FilDis;
    bool guille_enable = false, liu_enable = false, zono_enable = false;
    float alfa;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MuxObsNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}