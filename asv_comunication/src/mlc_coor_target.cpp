#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"            //Interface ref llc
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/reference_llc.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;


class MlcCoorTargetNode : public rclcpp::Node
{
public:
    MlcCoorTargetNode() : Node("mlc_coor_target") 
    {    

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(100),
                std::bind(&MlcCoorTargetNode::calculateTargetPose, this));
        subscriber_mavros_state = this-> create_subscription<mavros_msgs::msg::State>("/ASV4/mavros/state",1,
                std::bind(&MlcCoorTargetNode::callbackMavrosState, this, std::placeholders::_1));

        // Suscriptores para ASV0, ASV1 y ASV3
        subscriber_utar0 = this->create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/ASV0/control/reference_llc", 1,
            [this](const asv_interfaces::msg::ReferenceLlc::SharedPtr msg){
                callbackErrorMlc(msg, 0);
            });

        subscriber_utar1 = this->create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/ASV1/control/reference_llc", 1,
            [this](const asv_interfaces::msg::ReferenceLlc::SharedPtr msg){
                callbackErrorMlc(msg, 1);
            });

        subscriber_utar3 = this->create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/ASV3/control/reference_llc", 1,
            [this](const asv_interfaces::msg::ReferenceLlc::SharedPtr msg){
                callbackErrorMlc(msg, 3);
            });
        
        subscriber_utar4 = this->create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/ASV4/control/reference_llc", 1,
            [this](const asv_interfaces::msg::ReferenceLlc::SharedPtr msg){
                callbackErrorMlc(msg, 4);
            });



        publisher_center0 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV0/control/center_pose",1);
        publisher_target0 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV0/control/target_pose",1);

        publisher_center1 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV1/control/center_pose",1);
        publisher_target1 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV1/control/target_pose",1);

        publisher_center3 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV3/control/center_pose",1);
        publisher_target3 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV3/control/target_pose",1);

        publisher_center4 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV4/control/center_pose",1);
        publisher_target4 = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/ASV4/control/target_pose",1);


    	RCLCPP_INFO(this->get_logger(), "Target position Coordinate Node has been started.");
    }

private:

    void callbackErrorMlc(const asv_interfaces::msg::ReferenceLlc::SharedPtr msg, int drone_id)
    {
        // RCLCPP_INFO(this->get_logger(), "Received from %i",drone_id);
        if (drone_id == 0)
        {
            u_tar_act0 = msg->u_tar.data;
            std::vector<float> path = spatial_path(w0);
            float dx = path[3];
            float dy = path[4];
            w_dot0 = u_tar_act0 / (std::sqrt(dx*dx + dy*dy));
        }else if (drone_id == 1)
        {
            u_tar_act1 = msg->u_tar.data;
            std::vector<float> path = spatial_path(w1);
            float dx = path[3];
            float dy = path[4];
            w_dot1 = u_tar_act1 / (std::sqrt(dx*dx + dy*dy));
        }else if (drone_id == 3)
        {
            u_tar_act3 = msg->u_tar.data;
            std::vector<float> path = spatial_path(w3);
            float dx = path[3];
            float dy = path[4];
            w_dot3 = u_tar_act3 / (std::sqrt(dx*dx + dy*dy));
        }else if (drone_id == 4)
        {
            u_tar_act4 = msg->u_tar.data;
            std::vector<float> path = spatial_path(w4);
            float dx = path[3];
            float dy = path[4];
            w_dot4 = u_tar_act4 / (std::sqrt(dx*dx + dy*dy));
        }
    }

    void calculateTargetPose(){
        if(armed==false){
            w0 = 0;
            u_tar_act0 = 0;
            w_dot0 = 0;
            w1 = 0;
            u_tar_act1 = 0;
            w_dot1 = 0;
            w3 = 0;
            u_tar_act3 = 0;
            w_dot3 = 0;
            w4 = 0;
            u_tar_act4 = 0;
            w_dot4 = 0;
        }else{

            auto msg_pose = geometry_msgs::msg::PoseStamped();

            std::vector<float> path = spatial_path(w0);

            // Obtener puntos de trayectoria
            float xc = path[0];
            float yc = path[1];
            float phic = path[2];
            float dxc = path[3];
            float dyc = path[4];
            float dphic = path[5];

            float xp = xc + rho0 * cos(phic+theta0);  
            float yp = yc + rho0 * sin(phic+theta0);
            float phip = atan2 (dyc + dphic*(rho0 * cos(phic+theta0)), dxc + dphic*(rho0 * sin(phic+theta0)));

            // Send the pose base_link
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
            publisher_target0->publish(msg_pose); 

            msg_pose.pose.position.x= xc;
            msg_pose.pose.position.y= yc;
            msg_pose.pose.position.z= 0.0;
            q.setRPY(0, 0, phic);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_center0->publish(msg_pose); 

            path = spatial_path(w1);

            // Obtener puntos de trayectoria
            xc = path[0];
            yc = path[1];
            phic = path[2];
            dxc = path[3];
            dyc = path[4];
            dphic = path[5];

            xp = xc + rho1 * cos(phic+theta1);  
            yp = yc + rho1 * sin(phic+theta1);
            phip = atan2 (dyc + dphic*(rho1 * cos(phic+theta1)), dxc + dphic*(rho1 * sin(phic+theta1)));

            // Send the pose base_link
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= xp;
            msg_pose.pose.position.y= yp;
            msg_pose.pose.position.z= 0.0;

            q.setRPY(0, 0, phip);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_target1->publish(msg_pose); 

            msg_pose.pose.position.x= xc;
            msg_pose.pose.position.y= yc;
            msg_pose.pose.position.z= 0.0;
            q.setRPY(0, 0, phic);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_center1->publish(msg_pose); 

            path = spatial_path(w3);

            // Obtener puntos de trayectoria
            xc = path[0];
            yc = path[1];
            phic = path[2];
            dxc = path[3];
            dyc = path[4];
            dphic = path[5];

            xp = xc + rho3 * cos(phic+theta3);  
            yp = yc + rho3 * sin(phic+theta3);
            phip = atan2 (dyc + dphic*(rho3 * cos(phic+theta3)), dxc + dphic*(rho3 * sin(phic+theta3)));

            // Send the pose base_link
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= xp;
            msg_pose.pose.position.y= yp;
            msg_pose.pose.position.z= 0.0;

            q.setRPY(0, 0, phip);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_target3->publish(msg_pose); 

            msg_pose.pose.position.x= xc;
            msg_pose.pose.position.y= yc;
            msg_pose.pose.position.z= 0.0;
            q.setRPY(0, 0, phic);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_center3->publish(msg_pose); 

            path = spatial_path(w4);

            // Obtener puntos de trayectoria
            xc = path[0];
            yc = path[1];
            phic = path[2];
            dxc = path[3];
            dyc = path[4];
            dphic = path[5];

            xp = xc + rho4 * cos(phic+theta4);  
            yp = yc + rho4 * sin(phic+theta4);
            phip = atan2 (dyc + dphic*(rho4 * cos(phic+theta4)), dxc + dphic*(rho4 * sin(phic+theta4)));

            // Send the pose base_link
            msg_pose.header.stamp = this->now();
            msg_pose.header.frame_id = "map_ned";
            msg_pose.pose.position.x= xp;
            msg_pose.pose.position.y= yp;
            msg_pose.pose.position.z= 0.0;

            q.setRPY(0, 0, phip);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_target4->publish(msg_pose); 

            msg_pose.pose.position.x= xc;
            msg_pose.pose.position.y= yc;
            msg_pose.pose.position.z= 0.0;
            q.setRPY(0, 0, phic);
            msg_pose.pose.orientation.x = q.x();
            msg_pose.pose.orientation.y = q.y();
            msg_pose.pose.orientation.z = q.z();
            msg_pose.pose.orientation.w = q.w();
            publisher_center4->publish(msg_pose); 

            w0 = w0 +0.1*w_dot0;
            w1 = w1 +0.1*w_dot1;
            w3 = w3 +0.1*w_dot3;
            w4 = w4 +0.1*w_dot4;
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
                x_p  = w+5;  // 
                y_p  = w+5;    // 
                dx   = 1;     // 
                dy   = 1;     //
                ddx   = 0;     // 
                ddy   = 0;     //
                break;
            case 1:
                x_p  = 30-30*cos(w);  
                y_p  = 5+30*sin(w);     
                dx   = 30*sin(w);     
                dy   = 30*cos(w);   
                ddx   = 30*cos(w);     
                ddy   = -30*sin(w);    
                break;
        }
        float phi  = atan2(dy, dx);
        float dphi = (ddy*dx - ddx*dy)/(dx*dx + dy*dy);
        // Retornar un vector con las variables MX ordenadas
        return {x_p, y_p, phi, dx, dy, dphi};
    }

    bool armed = true;
    float w0 = 0, u_tar_act0 = 0, w_dot0 = 0;
    float w1 = 0, u_tar_act1 = 0, w_dot1 = 0;
    float w3 = 0, u_tar_act3 = 0, w_dot3 = 0;
    float w4 = 0, u_tar_act4 = 0, w_dot4 = 0;

    float rho0 = 4.0, theta0 = 3.141592; // -2.0944;
    float rho1 = 4.0, theta1 = -1.570796; // 0;
    float rho3 = 4.0, theta3 = 1.570796; // 2.0944;
    float rho4 = 4.0, theta4 =  0.0; // 2.0944;
    int path_d = 0;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar0;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar1;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar3;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_utar4;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_mavros_state;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_center0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_center1;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target3;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_center3;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_target4;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_center4;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MlcCoorTargetNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}