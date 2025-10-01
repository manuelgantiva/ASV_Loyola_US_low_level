#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"            //Interface ref llc
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/reference_llc.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;


class MlcTargetNode : public rclcpp::Node
{
public:
    MlcTargetNode() : Node("mlc_target") 
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
    
        my_id = (this->get_parameter("my_id").as_string());

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&MlcTargetNode::calculateTargetPose, this));
        subscriber_utar = this-> create_subscription<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1,
                std::bind(&MlcTargetNode::callbackErrorMlc, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&MlcTargetNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_target = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/control/target_pose",1);
    	RCLCPP_INFO(this->get_logger(), "Target position Node in %s has been started.", my_id.c_str());
    }

private:

    void callbackErrorMlc(const asv_interfaces::msg::ReferenceLlc::SharedPtr msg)
    {
        u_tar_act = msg->u_tar.data;
        float dxp = 1;  // 30*sin(w);  //  
        float dyp = 0;  // 30*cos(w);  //  
        w_dot = u_tar_act / (std::sqrt(dxp*dxp + dyp*dyp));
    }

    void calculateTargetPose(){
        if(armed==false){
            w = 0;
            u_tar_act = 0;
            w_dot = 0;
        }else{
            float xp  = w+10;  // 30-30*cos(w); //   
            float yp  = 10;    // 30*sin(w);    //   
            float dxp = 1;     // 30*sin(w);    //   
            float dyp = 0;     // 30*cos(w);    //   
            float psip = atan2(dyp, dxp);
        
            // Send the pose base_link
            auto msg_pose = geometry_msgs::msg::PoseStamped();
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= xp;
            msg_pose.pose.position.y= yp;
            msg_pose.pose.position.z= 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, psip);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_target->publish(msg_pose); 
            w = w +0.1*w_dot;
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    bool armed = false;
    float w = 0, u_tar_act = 0, w_dot = 0;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MlcTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}