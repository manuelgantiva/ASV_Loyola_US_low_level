#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"            //Interface ref llc
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/reference_llc.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;


class MlcModTargetNode : public rclcpp::Node
{
public:
    MlcModTargetNode() : Node("mlc_mod_target") 
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV3");
    
        my_id = (this->get_parameter("my_id").as_string());

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&MlcModTargetNode::calculateTargetPose, this));
        subscriber_utar = this-> create_subscription<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1,
                std::bind(&MlcModTargetNode::callbackErrorMlc, this, std::placeholders::_1));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&MlcModTargetNode::callbackMavrosState, this, std::placeholders::_1));
        publisher_center = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/control/center_pose",1);
        publisher_target = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/control/target_pose",1);
    	RCLCPP_INFO(this->get_logger(), "Target position Modified Node in %s has been started.", my_id.c_str());
    }

private:

    void callbackErrorMlc(const asv_interfaces::msg::ReferenceLlc::SharedPtr msg)
    {
        u_tar_act = msg->u_tar.data;
        std::vector<float> path = spatial_path(w);
        float dx = path[3];
        float dy = path[4];
        w_dot = u_tar_act / (std::sqrt(dx*dx + dy*dy));
    }

    void calculateTargetPose(){
        if(armed==false){
            w = 0;
            u_tar_act = 0;
            w_dot = 0;
        }else{

            std::vector<float> path = spatial_path(w);

            // Obtener puntos de trayectoria
            float xc = path[0];
            float yc = path[1];
            float phic = path[2];
            float dxc = path[3];
            float dyc = path[4];
            float dphic = path[5];

            float xp = xc + rho * cos(phic+theta);  
            float yp = yc + rho * sin(phic+theta);
            float phip = atan2 (dyc + dphic*(rho * cos(phic+theta)), dxc + dphic*(rho * sin(phic+theta)));

            // Send the pose base_link
            auto msg_pose = geometry_msgs::msg::PoseStamped();
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= xp;
            msg_pose.pose.position.y= yp;
            msg_pose.pose.position.z= 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, phip);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_target->publish(msg_pose); 

            msg_pose.pose.position.x= xc;
            msg_pose.pose.position.y= yc;
            msg_pose.pose.position.z= 0.0;
            q.setRPY(0, 0, phic);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_center->publish(msg_pose); 

            w = w +0.1*w_dot;
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    std::vector<float> spatial_path(const float& w) {
        float x_p, y_p, dx, dy, ddx, ddy;
        switch(path_d) {
            case 0:
                x_p  = w+7;  // 
                y_p  = w+7;    // 
                dx   = 1;     // 
                dy   = 1;     //
                ddx   = 0;     // 
                ddy   = 0;     //
                break;
            case 1:
                x_p  = 40-33*cos(w);  
                y_p  = 7+33*sin(w);     
                dx   = 33*sin(w);     
                dy   = 33*cos(w);   
                ddx   = 33*cos(w);     
                ddy   = -33*sin(w);    
                break;
        }
        float phi  = atan2(dy, dx);
        float dphi = (ddy*dx - ddx*dy)/(dx*dx + dy*dy);
        // Retornar un vector con las variables MX ordenadas
        return {x_p, y_p, phi, dx, dy, dphi};
    }

    bool armed = false;
    float w = 0, u_tar_act = 0, w_dot = 0;

    float rho = 6.0, theta = 1.570796;
    int path_d = 1;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_center;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MlcModTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}