#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/msg/rc_out.hpp"
#include "asv_interfaces/msg/state_observer.hpp"

class ObserverTestNode : public rclcpp::Node
{
public:
    ObserverTestNode() : Node("observer1")
    {
        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",rclcpp::SensorDataQoS(),
                std::bind(&ObserverTestNode::callbackImuData, this, std::placeholders::_1));
        subscriber_gps = this-> create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",rclcpp::SensorDataQoS(),
                std::bind(&ObserverTestNode::callbackGpsData, this, std::placeholders::_1));
        subscriber_compass = this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",rclcpp::SensorDataQoS(),
                std::bind(&ObserverTestNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/mavros/rc/out",1,
                std::bind(&ObserverTestNode::callbackRcoutData, this, std::placeholders::_1));
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer",1);
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(200),
                                          std::bind(&ObserverTestNode::calculateState, this));
        RCLCPP_INFO(this->get_logger(), "Observer Node has been started.");
    	
    }

private:
    void calculateState()
    {
        auto msg = asv_interfaces::msg::StateObserver();
        msg.point.x=1.0;
        msg.point.y=1.0;
        msg.point.z=1.0;
        msg.velocity.x=1.0;
        msg.velocity.y=1.0;
        msg.velocity.z=1.0;
        msg.disturbances.x=2.0;
        msg.disturbances.y=3.0;
        msg.disturbances.z=1.0;

        publisher_state->publish(msg);
    }

    void callbackImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        yaw=msg->angular_velocity.z;
        // RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);
    }

    void callbackGpsData(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        status_gps=msg->status.status;
        lat=msg->latitude;
        lon=msg->longitude;
        alt=msg->altitude;
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    void callbackCompassData(const std_msgs::msg::Float64::SharedPtr msg)
    {
        heading=msg->data;
        // RCLCPP_INFO(this->get_logger(), "Heading is: %f", heading);
    }

    void callbackRcoutData(const mavros_msgs::msg::RCOut::SharedPtr msg)
    {
        pwm_left=msg->channels[0];
        pwm_right=msg->channels[2];
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    float yaw, lat, lon, alt, heading;
    uint16_t pwm_left, pwm_right;
    int status_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}