#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/state.hpp"

class ImuFixNew : public rclcpp::Node
{
public:
    ImuFixNew() : Node("imu_fix_new")
    {     
        std::string my_id; 
        this->declare_parameter("my_id", "ASV0"); my_id = this->get_parameter("my_id").as_string();

        subscriber_imu = this->create_subscription<sensor_msgs::msg::Imu>( "/" + my_id + "/comunication/imu_ext/data",
            rclcpp::SensorDataQoS(),
            std::bind(&ImuFixNew::callbackImuData, this, std::placeholders::_1));
        
        subscriber_state = this->create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",
            1,
            std::bind(&ImuFixNew::callbackStateData, this, std::placeholders::_1));
        
        publisher_imu_extracted = this->create_publisher<sensor_msgs::msg::Imu>( "/" + my_id + "/control/imu_extracted", 1);
                                
        RCLCPP_INFO(this->get_logger(), "Imu Fix New Node in %s has been started.", my_id.c_str());    	
    }

private:
    
    void callbackImuData(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        if(armed == true)
        {
            // Extract only what you need from the IMU message
            auto extracted_msg = sensor_msgs::msg::Imu();
            
            // Header
            extracted_msg.header = msg->header;
            
            // Linear acceleration (3 values: x, y, z) in m/s^2
            extracted_msg.linear_acceleration.x = msg->linear_acceleration.x;
            extracted_msg.linear_acceleration.y = msg->linear_acceleration.y;
            extracted_msg.linear_acceleration.z = msg->linear_acceleration.z;
            
            // Angular velocity (3 values: x, y, z) in rad/s
            extracted_msg.angular_velocity.x = msg->angular_velocity.x;
            extracted_msg.angular_velocity.y = msg->angular_velocity.y;
            extracted_msg.angular_velocity.z = msg->angular_velocity.z;
            
            // Orientation quaternion (4 values: x, y, z, w)
            extracted_msg.orientation.x = msg->orientation.x;
            extracted_msg.orientation.y = msg->orientation.y;
            extracted_msg.orientation.z = msg->orientation.z;
            extracted_msg.orientation.w = msg->orientation.w;
            
            // Publish the extracted data
            publisher_imu_extracted->publish(extracted_msg);
            
            // Optional: Log the extracted data
            RCLCPP_INFO(this->get_logger(), 
                "Accel: [%.3f, %.3f, %.3f] m/s² | "
                "Gyro: [%.3f, %.3f, %.3f] rad/s | "
                "Quat: [%.3f, %.3f, %.3f, %.3f]",
                extracted_msg.linear_acceleration.x, 
                extracted_msg.linear_acceleration.y, 
                extracted_msg.linear_acceleration.z,
                extracted_msg.angular_velocity.x, 
                extracted_msg.angular_velocity.y, 
                extracted_msg.angular_velocity.z,
                extracted_msg.orientation.x, 
                extracted_msg.orientation.y, 
                extracted_msg.orientation.z, 
                extracted_msg.orientation.w);
        }
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed = msg->armed;
    }

    bool armed = true;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_imu_extracted;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFixNew>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
