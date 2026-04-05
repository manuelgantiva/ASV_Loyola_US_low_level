#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "nav_msgs/msg/odometry.hpp"                //Interface gps global local data
#include "std_msgs/msg/float32_multi_array.hpp"

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

using std::placeholders::_1;

class ObserverCoreNode : public rclcpp::Node
{
public:
    ObserverCoreNode() : Node("observer_core")
    {
        this-> declare_parameter("my_id", "ASV0");
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);


        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        t_s = Ts/1000; // En segundos

        Dz_up  = this->get_parameter("Dz_up").as_double();
        Dz_down = this->get_parameter("Dz_down").as_double();


        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts)),
                                          std::bind(&ObserverCoreNode::calculateState, this), cb_group_obs_);

        // suscribir a IMU 
        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/" + my_id + "/comunication/imu_ext/data",
                rclcpp::SensorDataQoS(), std::bind(&ObserverCoreNode::callbackImuData,
                this, std::placeholders::_1), options_sensors_);
        subscriber_gps_local= this-> create_subscription<nav_msgs::msg::Odometry>("/" + my_id + "/mavros/global_position/local",
                rclcpp::SensorDataQoS(), std::bind(&ObserverCoreNode::callbackGpsLocalData, this, std::placeholders::_1), options_sensors_);
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/" + my_id + "/mavros/rc/out",1,
                std::bind(&ObserverCoreNode::callbackRcoutData, this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&ObserverCoreNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        publisher_data = this->create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/data_sensors", rclcpp::SensorDataQoS());
        
                                          
        RCLCPP_INFO(this->get_logger(), "Observer Core Node in %s has been started.", my_id.c_str());
    }

private:
    void calculateState()
    {
        if(armed==false){ 
            count=0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                Yp.setZero();
                delta_diff=0;
                delta_mean=0;
                delta_l=0;
                delta_r=0;
                beta=0;
                psi = 0.0;
                r = 0.0;
            }
        }else{
            if(count > -1){

                Vector <double, 2> Yp_i;
                Vector <double, 2> Yr_i;
                float delta_diff_i;
                float delta_mean_i;
                float delta_l_i;
                float delta_r_i;
                int beta_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Yr_i << psi,
                            r;
                    Yp_i = Yp;
                    delta_diff_i = delta_diff;
                    delta_mean_i = delta_mean;
                    delta_l_i    = delta_l;
                    delta_r_i    = delta_r;
                    beta_i=beta;
                }

                auto message = std_msgs::msg::Float32MultiArray();

                message.data = {float(Yp_i[0]), float(Yp_i[1]), float(Yr_i[0]), float(Yr_i[1]), delta_diff_i, delta_mean_i,delta_l_i,delta_r_i, float(beta_i)};
                publisher_data->publish(message);
                
            }else{
                count=count+1;
            }
        }
    }

    // void callaback IMU
    void callbackImuData( const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if(armed==true){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                r = -1*msg->angular_velocity.z;                
            }   
        }
    }

    void callbackGpsLocalData(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(armed==true){
            float y = msg->pose.pose.position.x;
            float x = msg->pose.pose.position.y;
            float psi_rad = quat2EulerAngles_XYZ(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                                msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
            psi_rad=-psi_rad+(M_PI/2);
            if (psi_rad<0){
                psi_rad=psi_rad+(2*M_PI);
            }

            float dy = -0.2750;            //distancia de la antena del GPS al navio coordenada x
            float dx = 0.2625;           //distancia de la antena del GPS al navio coordenada y
            // 1) Xp = Xo + R(psi)*OP
            x = x + cos(psi_rad)*dx - sin(psi_rad)*dy;
            y = y + sin(psi_rad)*dx + cos(psi_rad)*dy;
            if(armed_act==false){
                psi_ant = psi_rad;
                laps = 0;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Yp << x,
                        y;
                    psi = psi_rad;
                }
            }else{
                psi_act = psi_rad;
                if((psi_act - psi_ant) > M_PI){
                    laps = laps - 1;
                }else if((psi_act - psi_ant) < -M_PI){
                    laps = laps + 1;
                }
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Yp << x,
                        y;
                    psi = psi_act + 2*M_PI*laps;
                }
                psi_ant=psi_act;
            }
        }else{
            psi_ant=0;
            laps=0;
        }
        armed_act=armed;
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

    void callbackRcoutData(const mavros_msgs::msg::RCOut::SharedPtr msg)
    {
        if(armed==true){
            uint16_t pwm_left=msg->channels[2];
            uint16_t pwm_right=msg->channels[0];
            int beta_a = 0;
            if(pwm_left>=1500 && pwm_right>=1500){
                beta_a=1;
            }else{
                beta_a=0;
            }
            float delta_left =  ((pwm_left/400.0) -3.75);    //normalized pwm
            float delta_right = ((pwm_right/400.0) -3.75);  //normalized pwm

            delta_left = deleteDeadZone(delta_left);
            delta_right = deleteDeadZone(delta_right);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                delta_diff = delta_left-delta_right;
                delta_mean = (delta_left+delta_right)/2.0;
                delta_l    = delta_left;
                delta_r    = delta_right;
                beta=beta_a;
            }
        }
    }

    float deleteDeadZone(float delta)
    {
        if(delta>Dz_down && delta<Dz_up){
            delta = 0;
        }else if(delta <= Dz_down){
            delta = delta-Dz_down;
        }else if(delta >= Dz_up){
            delta = delta-Dz_up;
        }
        return delta;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    
    float psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;
    float r = 0.0, x = 0.0, y = 0.0;

 //------Params-------//
    std::string my_id;
    float Ts, t_s;
    float Dz_up, Dz_down;  


    float delta_diff;
    float delta_mean;
    float delta_l;
    float delta_r;
    int beta, count=0;

    Vector <double, 2> Yp;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_data;

    rclcpp::TimerBase::SharedPtr timer_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverCoreNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
