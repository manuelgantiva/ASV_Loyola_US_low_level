#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface compass initial psi

#include <cmath>

#define PI 3.141592

using std::placeholders::_1;


class ReferenceVelNode : public rclcpp::Node
{
public:
    ReferenceVelNode() : Node("reference_vel") 
    {
        this-> declare_parameter("reference_mode", false);
        reference_mode_ = this->get_parameter("reference_mode").as_bool();

        subscriber_compass = this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",rclcpp::SensorDataQoS(),
                std::bind(&ReferenceVelNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_rc_in = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",1,
                std::bind(&ReferenceVelNode::callbackRcIn, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ReferenceVelNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_ref = this-> create_publisher<geometry_msgs::msg::Vector3>("/control/reference_llc",1);
    	RCLCPP_INFO(this->get_logger(), "Reference Vel Node has been started.");
    }

private:

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
    {
        if(armed==true){
            float u_ref=0.0;
            float r_ref=0.0;
            if(msg->channels[2] < 1550 && msg->channels[2] > 1450){
                msg->channels[2] = 1500;
            }
            if(msg->channels[0] < 1550 && msg->channels[0] > 1450){
                msg->channels[0] = 1500;
            }

            u_ref = ((msg->channels[2]*0.0025)-3.75);
            r_ref = ((msg->channels[0]*0.0025)-3.75);

            auto ref = geometry_msgs::msg::Vector3();
            if(u_ref>1.0){
                u_ref = 1.0;
            }else if (u_ref<=-1.0)
            {
                u_ref = -1.0;
            }
            if(r_ref>1.0){
                r_ref = 1.0;
            }else if (r_ref<=-1.0)
            {
                r_ref = -1.0;
            }
            
            if(reference_mode_ == true){
                ref.x = u_ref;
                ref.y = r_ref;
            }else{
                float periodo = 10.0; // Período en segundos
                u_ref = u_ref/2;
                r_ref = r_ref/2;
                ref.x = u_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+u_ref;
                ref.y = r_ref*(sin((2 * PI / periodo) * (counter_*0.1)))+r_ref;
                counter_++;
            }
                        
            psi_act = psi_act+(ref.y*0.1);
            ref.z = psi_act;
            publisher_ref->publish(ref);
        }
    }

    void callbackCompassData(const std_msgs::msg::Float64::SharedPtr msg)
    {
        if(msg->data > 180.0){
            init_psi = (msg->data*PI/180)-(2*PI);
        }else{
            init_psi = (msg->data*PI/180);
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if(msg->armed == true && msg->armed!=armed){
            counter_ = 0;
            psi_act = init_psi;
            RCLCPP_INFO(this->get_logger(), "Initial psi= %f", psi_act);
        }
        armed= msg->armed;
    }

    bool armed = false;
    float init_psi = 0.0, psi_act = 0.0;
    bool reference_mode_;
    int counter_=0;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_rc_in;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_ref;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReferenceVelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}