#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/xbee_observer.hpp"     //Interface transceiver xbee data
#include "asv_interfaces/msg/state_neighbor.hpp"     //Interface neighbor state data
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
        subscriber_xbee = this-> create_subscription<asv_interfaces::msg::XbeeObserver>(
            "/comunication/xbee_observer",1, 
            std::bind(&FramePublisher::callbackXbeeData, this, std::placeholders::_1));
        publisher_pose_neighbor = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/control/pose_neighbor",
                rclcpp::SensorDataQoS());
        publisher_pose = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/control/pose",
                rclcpp::SensorDataQoS());
    	RCLCPP_INFO(this->get_logger(), "Frame Publisher Node has been started.");

    }

private:
    
    void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (status_gps!=-1){
            geometry_msgs::msg::TransformStamped t;

            // corresponding tf variables
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map";
            t.child_frame_id = "gps_link";

            // asv only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = msg->pose.position.x;
            t.transform.translation.y = msg->pose.position.y;
            t.transform.translation.z = 0.5;

            t.transform.rotation.x = msg->pose.orientation.x;
            t.transform.rotation.y = msg->pose.orientation.y;
            t.transform.rotation.z = msg->pose.orientation.z;
            t.transform.rotation.w = msg->pose.orientation.w;

            // Send the transformation
            tf_broadcaster_->sendTransform(t);

            // Send the pose base_link
            float y = msg->pose.position.x;
            float x = msg->pose.position.y;
            float psi_rad = quat2EulerAngles_XYZ(msg->pose.orientation.w, msg->pose.orientation.x,
                                                msg->pose.orientation.y, msg->pose.orientation.z);
            psi_rad=-psi_rad+1.5708;
            float dy = -0.2750;            //distancia de la antena del GPS al navio coordenada x
            float dx = 0.2625;           //distancia de la antena del GPS al navio coordenada y
            // 1) Xp = Xo + R(psi)*OP
            x = x + cos(psi_rad)*dx - sin(psi_rad)*dy;
            y = y + sin(psi_rad)*dx + cos(psi_rad)*dy;

            auto msg_pose = geometry_msgs::msg::PoseStamped();
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= x;
            msg_pose.pose.position.y= y;
            msg_pose.pose.position.z= 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, psi_rad);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_pose->publish(msg_pose); 

        }
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    void callbackXbeeData(const asv_interfaces::msg::XbeeObserver::SharedPtr msg)
    {
        if (msg->counter > 0){
            geometry_msgs::msg::TransformStamped t;
            
            // corresponding tf variables
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "map_ned";
            t.child_frame_id = "ASV0";

            // asv only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            t.transform.translation.x = msg->states[0].point.x;
            t.transform.translation.y = msg->states[0].point.y;
            t.transform.translation.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, msg->states[0].point.z);

            t.transform.rotation.x = q.x();
            t.transform.rotation.y = q.y();
            t.transform.rotation.z = q.z();
            t.transform.rotation.w = q.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t);

             // Public Pose vector
            auto msg_pose = geometry_msgs::msg::PoseStamped();

            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= msg->states[0].point.x;
            msg_pose.pose.position.y= msg->states[0].point.y;
            msg_pose.pose.position.z= 0.0;

            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_pose_neighbor->publish(msg_pose); 
        }
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    float quat2EulerAngles_XYZ(float q0, float q1, float q2,float q3)
    {
        const double q0_2 = q0 * q0;
        const double q1_2 = q1 * q1;
        const double q2_2 = q2 * q2;
        const double q3_2 = q3 * q3;
        const double x2q1q2 = 2.0 * q1 * q2;
        const double x2q0q3 = 2.0 * q0 * q3;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double psi = atan2(m12, m11);
        return static_cast<float>(psi);
    }


    int status_gps;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<asv_interfaces::msg::XbeeObserver>::SharedPtr subscriber_xbee;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_neighbor;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose;

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