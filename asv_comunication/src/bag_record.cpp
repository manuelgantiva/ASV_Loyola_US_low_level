#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/xbee_observer.hpp"    //Interface xbee observer
#include "geometry_msgs/msg/twist_stamped.hpp"       //Interface velocity

#include <rosbag2_cpp/writer.hpp>
#include <filesystem>
#include <ctime>

namespace fs = std::filesystem;

class BagRecordNode : public rclcpp::Node 
{
public:
    BagRecordNode() : Node("bag_record") 
    {
        // Obtener la fecha y hora actual
        std::time_t now = std::time(0);
        std::tm *local_time = std::localtime(&now);
        int day = local_time->tm_mday;   // Día del mes (1-31)
        int month = local_time->tm_mon + 1;  // Mes (0-11, agregamos 1 para obtener el mes real)
        prefix = std::to_string(day) + "-" + std::to_string(month) + "-bag" + "-";
        RCLCPP_INFO(this->get_logger(), "Current day: %s", prefix.c_str());

        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",10,
                std::bind(&BagRecordNode::callbackMavrosState, this, std::placeholders::_1));
        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",rclcpp::SensorDataQoS(),
                std::bind(&BagRecordNode::callbackImuData, this, std::placeholders::_1));
        subscriber_gps_global = this-> create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackGpsGlobalData, this, std::placeholders::_1));
        subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackGpsLocalData, this, std::placeholders::_1));
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/mavros/rc/out",1,
                std::bind(&BagRecordNode::callbackRcoutData, this, std::placeholders::_1));
        subscriber_state_guille= this-> create_subscription<asv_interfaces::msg::StateObserver>("/control/state_observe_guille",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackStateGuilleData, this, std::placeholders::_1));
        subscriber_state_liu= this-> create_subscription<asv_interfaces::msg::StateObserver>("/control/state_observe_liu",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackStateLiulData, this, std::placeholders::_1));
        subscriber_pose= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseData, this, std::placeholders::_1));
        subscriber_pose_guille= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose_guille",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseGuilleData, this, std::placeholders::_1));
        subscriber_pose_liu= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose_liu",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseLiuData, this, std::placeholders::_1));
        subscriber_compass= this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_vel_body= this-> create_subscription<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_body",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackVelocityBodyData, this, std::placeholders::_1));
        subscriber_vel_local= this-> create_subscription<geometry_msgs::msg::TwistStamped>("mavros/local_position/velocity_local",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackVelocityLocalData, this, std::placeholders::_1));
        subscriber_xbee= this-> create_subscription<asv_interfaces::msg::XbeeObserver>("/comunication/xbee_observer",1,
                std::bind(&BagRecordNode::callbackXbeeData, this, std::placeholders::_1));

    	RCLCPP_INFO(this->get_logger(), "Bag Record Node has been started.");
    }

private:
    void callbackXbeeData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/comunication/xbee_observer", "asv_interfaces/msg/XbeeObserver", time_stamp);
        }
    }

    void callbackVelocityBodyData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "mavros/local_position/velocity_body", "geometry_msgs/msg/TwistStamped", time_stamp);
        }
    }

    void callbackVelocityLocalData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "mavros/local_position/velocity_local", "geometry_msgs/msg/TwistStamped", time_stamp);
        }
    }

    void callbackCompassData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/global_position/compass_hdg", "std_msgs/msg/Float64", time_stamp);
        }
    }

    void callbackPoseData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pose", "geometry_msgs/msg/PoseStamped", time_stamp);
        }
    }

    void callbackPoseGuilleData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pose_guille", "geometry_msgs/msg/PoseStamped", time_stamp);
        }
    }

    void callbackPoseLiuData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pose_liu", "geometry_msgs/msg/PoseStamped", time_stamp);
        }
    }

    void callbackStateGuilleData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/state_observe_guille", "asv_interfaces/msg/StateObserver", time_stamp);
        }
    }

    void callbackStateLiulData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/state_observe_liu", "asv_interfaces/msg/StateObserver", time_stamp);
        }
    }

    void callbackGpsGlobalData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/global_position/global", "sensor_msgs/msg/NavSatFix", time_stamp);
        }
    }

    void callbackGpsLocalData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/local_position/pose", "geometry_msgs/msg/PoseStamped", time_stamp);
        }
    }

    void callbackRcoutData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/rc/out", "mavros_msgs/msg/RCOut", time_stamp);
        }
    }

    void callbackImuData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/imu/data", "sensor_msgs/msg/Imu", time_stamp);
        }
    }

    void callbackMavrosState(const mavros_msgs::msg::State::SharedPtr msg)
    {
        //RCLCPP_INFO(this->get_logger(), "mode: %s, manual input: %d, armed: %d", msg->mode.c_str(),
                        //msg->manual_input, msg->armed);
        if(armed != msg->armed){
            if(msg->armed == true){
                std::string current_directory = fs::current_path();
                int max_bag = 0;
                for (const auto &entry : fs::directory_iterator(current_directory))
                {
                    if (entry.is_directory()    )
                    {
                        std::string directory_name = entry.path().filename().string();
                        if (directory_name.compare(0, prefix.length(), prefix) == 0) {
                            int bag_num = std::stoi(directory_name.substr(prefix.length()));
                            if(bag_num >= max_bag){
                                max_bag = bag_num+1;
                            }
                        } 
                    }
                }
                name_bag = prefix + std::to_string(max_bag);
                writer_ = std::make_unique<rosbag2_cpp::Writer>();
                writer_->open(name_bag);
            }else{
                writer_->close();
            }
        }
        armed= msg->armed;
    }

    std::string name_bag;
    bool armed = false;
    std::string prefix;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_global;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_liu;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_liu;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_guille;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_guille;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_vel_local;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_vel_body;
    rclcpp::Subscription<asv_interfaces::msg::XbeeObserver>::SharedPtr subscriber_xbee;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BagRecordNode>(); 
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}