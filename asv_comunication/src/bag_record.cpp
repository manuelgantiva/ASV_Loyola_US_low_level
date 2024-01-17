#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/rc_in.hpp"                //Interface rc inputs
#include "mavros_msgs/msg/override_rc_in.hpp"       //Interface rc override inputs
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "mavros_msgs/msg/state.hpp"                //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/xbee_observer.hpp"     //Interface xbee observer
#include "geometry_msgs/msg/twist_stamped.hpp"       //Interface velocity
#include "geometry_msgs/msg/twist.hpp"              //Interface cmd vel ardupilot
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override


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
        this-> declare_parameter("my_id", 0);
        my_id = std::to_string(this->get_parameter("my_id").as_int());
        std::time_t now = std::time(0);
        std::tm *local_time = std::localtime(&now);
        int day = local_time->tm_mday;   // Día del mes (1-31)
        int month = local_time->tm_mon + 1;  // Mes (0-11, agregamos 1 para obtener el mes real)
        prefix = "ASV" + my_id + "-" + std::to_string(day) + "-" + std::to_string(month) + "-bag" + "-";
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
        subscriber_rcin = this-> create_subscription<mavros_msgs::msg::RCIn>("/mavros/rc/in",1,
                std::bind(&BagRecordNode::callbackRcinData, this, std::placeholders::_1));
        subscriber_rc_over_in = this-> create_subscription<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override",1,
                std::bind(&BagRecordNode::callbackRcOverinData, this, std::placeholders::_1));
        subscriber_reference = this-> create_subscription<geometry_msgs::msg::Vector3>("/control/reference_llc",1,
                std::bind(&BagRecordNode::callbackReference, this, std::placeholders::_1));
        subscriber_cmd_vel = this-> create_subscription<geometry_msgs::msg::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped"
                ,1, std::bind(&BagRecordNode::callbackCmdVel, this, std::placeholders::_1));
        subscriber_ifac_pwm = this-> create_subscription<asv_interfaces::msg::PwmValues>("/control/pwm_value_ifac",rclcpp::SensorDataQoS(),
                std::bind(&BagRecordNode::callbackIfacPwm, this, std::placeholders::_1));
        subscriber_pwm = this-> create_subscription<asv_interfaces::msg::PwmValues>("/control/pwm_values",1,
                std::bind(&BagRecordNode::callbackPwms, this, std::placeholders::_1));
        subscriber_state_guille= this-> create_subscription<asv_interfaces::msg::StateObserver>("/control/state_observer_guille",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackStateGuilleData, this, std::placeholders::_1));
        subscriber_state_liu= this-> create_subscription<asv_interfaces::msg::StateObserver>("/control/state_observer_liu",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackStateLiulData, this, std::placeholders::_1));
        subscriber_states= this-> create_subscription<asv_interfaces::msg::StateObserver>("/control/state_observer",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackStateData, this, std::placeholders::_1));
        subscriber_pose= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseData, this, std::placeholders::_1));
        subscriber_pose_neighbor= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose_neighbor",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseNeighborData, this, std::placeholders::_1));
        subscriber_pose_guille= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose_guille",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseGuilleData, this, std::placeholders::_1));
        subscriber_pose_liu= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/control/pose_liu",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackPoseLiuData, this, std::placeholders::_1));
        subscriber_compass= this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_vel_body= this-> create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/local_position/velocity_body",
                rclcpp::SensorDataQoS(), std::bind(&BagRecordNode::callbackVelocityBodyData, this, std::placeholders::_1));
        subscriber_vel_local= this-> create_subscription<geometry_msgs::msg::TwistStamped>("/mavros/local_position/velocity_local",
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
            writer_->write(msg, "/mavros/local_position/velocity_body", "geometry_msgs/msg/TwistStamped", time_stamp);
        }
    }

    void callbackVelocityLocalData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/local_position/velocity_local", "geometry_msgs/msg/TwistStamped", time_stamp);
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

    void callbackPoseNeighborData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pose_neighbor", "geometry_msgs/msg/PoseStamped", time_stamp);
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
            writer_->write(msg, "/control/state_observer_guille", "asv_interfaces/msg/StateObserver", time_stamp);
        }
    }

    void callbackStateLiulData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/state_observer_liu", "asv_interfaces/msg/StateObserver", time_stamp);
        }
    }

    void callbackStateData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/state_observer", "asv_interfaces/msg/StateObserver", time_stamp);
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

    void callbackRcinData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/rc/in", "mavros_msgs/msg/RCIn", time_stamp);
        }
    }

    void callbackRcOverinData(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/rc/override", "mavros_msgs/msg/OverrideRCIn", time_stamp);
        }
    }

    void callbackReference(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/reference_llc", "geometry_msgs/msg/Vector3", time_stamp);
        }
    }

    void callbackCmdVel(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/mavros/setpoint_velocity/cmd_vel_unstamped", "geometry_msgs/msg/Twist", time_stamp);
        }
    }

    void callbackIfacPwm(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pwm_value_ifac", "asv_interfaces/msg/PwmValues", time_stamp);
        }
    }

    void callbackPwms(const std::shared_ptr<rclcpp::SerializedMessage> msg) 
    {
        if(armed==true){
            rclcpp::Time time_stamp = this->now();
            writer_->write(msg, "/control/pwm_values", "asv_interfaces/msg/PwmValues", time_stamp);
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

    std::string my_id;
    std::string name_bag;
    bool armed = false;
    std::string prefix;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_global;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_rcin;
    rclcpp::Subscription<mavros_msgs::msg::OverrideRCIn>::SharedPtr subscriber_rc_over_in;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_reference;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd_vel;
    rclcpp::Subscription<asv_interfaces::msg::PwmValues>::SharedPtr subscriber_ifac_pwm;
    rclcpp::Subscription<asv_interfaces::msg::PwmValues>::SharedPtr subscriber_pwm;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_liu;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_liu;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_guille;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_state_guille;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_neighbor;
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