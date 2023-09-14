#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <string>
#include <cmath>

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher() : Node("asv_tf2_broadcaster")
    {
        // Declare and acquire 'asv_id' parameter
        asv_id_= this-> declare_parameter<std::string>("asv_id", "asv0");

        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/local_position/pose",rclcpp::SensorDataQoS(), 
            std::bind(&FramePublisher::callbackGpsLocalData, this, std::placeholders::_1));
    	RCLCPP_INFO(this->get_logger(), "Frame Publisher Node has been started.");
    }

private:
    void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (status_gps!=-1){
            float x=msg->pose.position.x;
            float y=msg->pose.position.y;
            geometry_msgs::msg::TransformStamped t;
            tf2::Quaternion q;

            // corresponding tf variables
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "gps_link";

            // asv only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = msg->pose.position.x;
            t.transform.translation.y = msg->pose.position.y;
            t.transform.translation.z = msg->pose.position.z;

            t.transform.rotation.x = msg->pose.orientation.x;
            t.transform.rotation.y = msg->pose.orientation.y;
            t.transform.rotation.z = msg->pose.orientation.z;
            t.transform.rotation.w = msg->pose.orientation.w;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
        }
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    int status_gps;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string asv_id_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FramePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}