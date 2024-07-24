#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "geometry_msgs/msg/twist.hpp"              //Interface accel computed

#include <cmath>
#include <thread>
#include <Eigen/Dense>

using namespace Eigen;

using std::placeholders::_1;

class ImuFixNode : public rclcpp::Node
{
public:
    ImuFixNode() : Node("imu_fix")
    {     
        //---------Parámetros del ASV-------------------//
        this-> declare_parameter("alpha", 0.15);
        
        alpha = this->get_parameter("alpha").as_double();

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ImuFixNode::param_callback, this, _1));

        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",rclcpp::SensorDataQoS(),
                std::bind(&ImuFixNode::callbackImuData, this, std::placeholders::_1));
        //subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
        //       rclcpp::SensorDataQoS(), std::bind(&ImuFixNode::callbackGpsLocalData, this, std::placeholders::_1));
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ImuFixNode::callbackStateData, this, std::placeholders::_1));
        
        publisher_accel = this-> create_publisher<geometry_msgs::msg::Twist>("/control/accel_imu",1);
        
        ema_previous << 0.0, 0.0, -9.7;
                                
        RCLCPP_INFO(this->get_logger(), "Imu Fix Node has been started.");
    	
    }

private:
    
    /*void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(armed==true){
            if (status_gps!=-1){
                float psi_rad = quat2EulerAngles_XYZ(msg->pose.orientation.w, msg->pose.orientation.x,
                                                    msg->pose.orientation.y, msg->pose.orientation.z);
                psi_rad=-psi_rad+(M_PI/2);
                if (psi_rad<0){
                    psi_rad=psi_rad+(2*M_PI);
                }
                RCLCPP_INFO(this->get_logger(), "Heading is: %f", psi_rad);
            }
        }else{
            psi_ant=0;
        }
        armed_act=armed;
    }*/

    void callbackImuData(const sensor_msgs::msg::Imu::SharedPtr msg) 
    {
        if(armed==true){
            // Obtener la orientación cuaternión del mensaje IMU
            auto q = msg->orientation;

             // Obtener el cuaternión 
            Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);

            // Pasar a ángulos de Euler
            Eigen::Vector3d euler = Cquat2EulerAngles_XYZ(q.w, q.x, q.y, q.z);

            double yaw = normalizeAngle(-euler[0]+(M_PI/2)); // Rotación alrededor del eje Z (psi)
            double pitch = -euler[1]; // Rotación alrededor del eje Y (theta)
            double roll = euler[2];  // Rotación alrededor del eje X (phi)

            Eigen::Matrix3d rotation_nb;
            rotation_nb = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

            Eigen::Matrix3d rotation_bp;
            rotation_bp = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

            // Definir la aceleración en el marco de referencia bcon Z hacia Arriba
            Eigen::Vector3d acceleration_b(msg->linear_acceleration.x,
                                        msg->linear_acceleration.y,
                                        msg->linear_acceleration.z); // Ejemplo de aceleración en b

            // Transformar la aceleración al marco de referencia n
            Eigen::Vector3d acceleration_n = rotation_nb * rotation_bp * acceleration_b;

            ema_previous = applyEMA(acceleration_n, ema_previous);
            
            auto msg = geometry_msgs::msg::Twist();
            msg.linear.x=ema_previous[0];
            msg.linear.y=ema_previous[1];
            msg.linear.z=ema_previous[2];
            msg.angular.x=roll;
            msg.angular.y=pitch;
            msg.angular.z=yaw;
            publisher_accel->publish(msg);  

            //RCLCPP_INFO(this->get_logger(), "ax: %f, ay: %f, az: %f", acceleration_n[0], acceleration_n[1],
            //                    acceleration_n[2]);   
            //RCLCPP_INFO(this->get_logger(), "Old Roll: %f, Pitch: %f, Yaw: %f", roll, pitch, yaw);             

        }else{
            ema_previous << 0.0, 0.0, -9.7;
        }
    }

    float normalizeAngle(float angle) {
        // Utiliza fmod para obtener el ángulo en el rango [-2π, 2π]
        angle = fmod(angle, 2 * M_PI);
        // Si el ángulo es negativo, ajustarlo para que esté en el rango [0, 2π]
        if (angle < 0) {
            angle += 2 * M_PI;
        }
        return angle;
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
        const double psic = atan2(m12, m11);        
        return static_cast<float>(psic);
    }

    Eigen::Vector3d Cquat2EulerAngles_XYZ(float q0, float q1, float q2,float q3)
    {
        const double q0_2 = q0 * q0;
        const double q1_2 = q1 * q1;
        const double q2_2 = q2 * q2;
        const double q3_2 = q3 * q3;
        const double x2q1q2 = 2.0 * q1 * q2;
        const double x2q0q3 = 2.0 * q0 * q3;
        const double x2q1q3 = 2.0 * q1 * q3;
        const double x2q0q2 = 2.0 * q0 * q2;
        const double x2q2q3 = 2.0 * q2 * q3;
        const double x2q0q1 = 2.0 * q0 * q1;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double m13 = x2q1q3 - x2q0q2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(m23, m33);
        const double theta = -asin(m13);
        const double psi = atan2(m12, m11);
        Eigen::Vector3d result(psi,theta,phi);
        return result;
    }

    Eigen::Vector3d applyEMA(const Eigen::Vector3d& current_acc, const Eigen::Vector3d& previous_ema) {
        return alpha * current_acc + (1.0 - alpha) * previous_ema;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "alpha"){
                if(param.as_double() >= 0.0 and param.as_double() <= 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    alpha = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-1");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }


    float psi_act = 0.0, psi_ant = 0.0, psi = 0.0;
    int status_gps;
    bool armed = false, armed_act = false;

    //------Params-------//
    float alpha;
    Eigen::Vector3d ema_previous;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_accel;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuFixNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}