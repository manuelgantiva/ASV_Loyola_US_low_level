#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
#define PI 3.141592

class ObserverZonoNode : public rclcpp::Node
{
public:
    ObserverZonoNode() : Node("observer_zono")
    {
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("m", 24.39);
        this-> declare_parameter("Xu", -1.0);
        this-> declare_parameter("Yv", -10.92);
        this-> declare_parameter("Nr", -3.95);
        this-> declare_parameter("xg", 0.0196);
        this-> declare_parameter("Iz", 1.81);
        
        Ts = this->get_parameter("Ts").as_double();
        m = this->get_parameter("m").as_double();
        Xu = this->get_parameter("Xu").as_double();
        Yv = this->get_parameter("Yv").as_double();
        Nr = this->get_parameter("Nr").as_double();
        xg = this->get_parameter("xg").as_double();
        Iz = this->get_parameter("Iz").as_double();


        M << m-Xu, 0.0, 0.0,
             0.0, m-Yv, m*xg,
             0.0, m*xg, Iz-Nr;  

        M_1 = M.inverse();  

        Apsi << 0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                0.0, 0.0, 0.0;     

        Bpsi << 0.0, 0.0, 0.0,
                M_1(2,0), M_1(2,1), M_1(2,2),
                0.0, 0.0, 0.0;

        Cpsi << 1.0, 0.0, 0.0;

        Ap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        Bp << 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0,
              M_1(0,0), M_1(0,1), M_1(0,2),
              M_1(1,0), M_1(1,1), M_1(1,2),
              0.0, 0.0, 0.0,
              0.0, 0.0, 0.0;

        Cp << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

        Tp << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        R2T.setZero();
        Yp.setZero();
        Xp_hat.setZero();
        Xp_hat_dot.setZero(); 
        Xp_hat_ant.setZero(); 
        Xpsi_hat.setZero(); 
        Xpsi_hat_dot.setZero(); 
        Xpsi_hat_ant.setZero(); 
        tao.setZero(); 

    
        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/mavros/imu/data",rclcpp::SensorDataQoS(),
                std::bind(&ObserverZonoNode::callbackImuData, this, std::placeholders::_1));
        subscriber_gps_global = this-> create_subscription<sensor_msgs::msg::NavSatFix>("/mavros/global_position/global",
                rclcpp::SensorDataQoS(), std::bind(&ObserverZonoNode::callbackGpsGlobalData, this, std::placeholders::_1));
        subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
                rclcpp::SensorDataQoS(), std::bind(&ObserverZonoNode::callbackGpsLocalData, this, std::placeholders::_1));
        subscriber_compass = this-> create_subscription<std_msgs::msg::Float64>("/mavros/global_position/compass_hdg",
                rclcpp::SensorDataQoS(), std::bind(&ObserverZonoNode::callbackCompassData, this, std::placeholders::_1));
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/mavros/rc/out",1,
                std::bind(&ObserverZonoNode::callbackRcoutData, this, std::placeholders::_1));
         subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ObserverZonoNode::callbackStateData, this, std::placeholders::_1));
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer",
                rclcpp::SensorDataQoS());
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts)),
                                          std::bind(&ObserverZonoNode::calculateState, this));
        RCLCPP_INFO(this->get_logger(), "Observer Zono Nodehas been started.");
    	
    }

private:
    void calculateState()
    {
        auto msg = asv_interfaces::msg::StateObserver();
        if(armed==false){
            Xp_hat.setZero();
            Xp_hat_dot.setZero(); 
            Xp_hat_ant.setZero(); 
            Xpsi_hat.setZero(); 
            Xpsi_hat_dot.setZero(); 
            Xpsi_hat_ant.setZero(); 
            R2T.setZero();
            Yp.setZero();
            tao.setZero(); 
        }else{
            auto start = std::chrono::high_resolution_clock::now();

            float cospsi= cos(psi);
            float senpsi= sin(psi);
            R2T << cospsi, senpsi,
                   -senpsi, cospsi;

            Tp(0,0)=cospsi;
            Tp(0,1)=senpsi;
            Tp(1,0)=-senpsi;
            Tp(1,1)=cospsi;

            Xpsi_hat.setZero(); 
            Xp_hat.setZero();

            //std::cout << "Matriz Xphd \n" << Xp_hat_ant << std::endl; 

            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.point.x=Xp_hat(0,0);
            msg.point.y=Xp_hat(1,0);
            msg.point.z=Xpsi_hat(0,0);
            msg.velocity.x=Xp_hat(2,0);
            msg.velocity.y=Xp_hat(3,0);
            msg.velocity.z=Xpsi_hat(1,0);
            msg.disturbances.x=Xp_hat(4,0);
            msg.disturbances.y=Xp_hat(5,0);
            msg.disturbances.z=Xpsi_hat(2,0);

            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            double miliseconds = elapsed.count()*1000;
            RCLCPP_INFO(this->get_logger(), "Exec time: %f", miliseconds);
        }
    

        publisher_state->publish(msg);
    }

    void callbackImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        yaw=msg->angular_velocity.z;
        //RCLCPP_INFO(this->get_logger(), "yaw: %f", yaw);
    }

    void callbackGpsGlobalData(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        status_gps=msg->status.status;
        if (status_gps!=-1){
            lat=msg->latitude;
            lon=msg->longitude;
            alt=msg->altitude;
            //RCLCPP_INFO(this->get_logger(), "gps lat is: %f, lon is: %f y alt is: %f", lat, lon, alt);
        }
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (status_gps!=-1){
            float x=msg->pose.position.x;
            float y=msg->pose.position.y;
            float dx = -0.2750;         //distancia en x desde la antena del GPS al navio (punto O)
            float dy = -0.2625 ;        //distancia en y desde la antena del GPS al navio (punto O)

             // Xp = Xo + R(psi)*OP
            x_centro = x + cos(psi)*dx - sin(psi)*dy;
            y_centro = y + sin(psi)*dx + cos(psi)*dy;
            Yp << x_centro,
                  y_centro;
            //RCLCPP_INFO(this->get_logger(), "gps x is: %f, y is: %f", x, y);
        }
        // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
    }

    void callbackCompassData(const std_msgs::msg::Float64::SharedPtr msg)
    {
        float psi_rad=msg->data*PI/180;
        if(armed==false){
            psi_0 = psi_rad;
            psi_ant = psi_0;
            laps = 0.0;
            psi = psi_rad;
        }else{
            psi_act = psi_rad;
            psi_act = psi_act - psi_0;

            if((psi_act - psi_ant) > PI){
                laps = laps - 1;
            }else if((psi_act - psi_ant) < -PI){
                laps = laps + 1;
            }
            psi = psi_act + 2*PI*laps;
            psi_ant=psi_act;
        }
        // RCLCPP_INFO(this->get_logger(), "n is: %d", int(laps));
        // RCLCPP_INFO(this->get_logger(), "Heading is: %f", psi);
    }

    void callbackRcoutData(const mavros_msgs::msg::RCOut::SharedPtr msg)
    {
        uint16_t pwm_left=msg->channels[0];
        uint16_t pwm_right=msg->channels[2];
        float Tr=0;
        float Tl=0;
        if(pwm_left>1550){
            Tl=((0.3125*pwm_left)-481.5)/2;
        }else if (pwm_left<1450)
        {
            Tl=((0.3125*pwm_left)-456.25)/2;
        }
        if(pwm_right>1550){
            Tr=((0.3125*pwm_right)-481.5)/2;
        }else if (pwm_right<1450)
        {
            Tr=((0.3125*pwm_right)-456.25)/2;
        }
        tao << Tr + Tl,
               0.0,
               (Tl - Tr)*0.68/2;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    float yaw, lat, lon, alt, psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0, x_centro, y_centro;
    int status_gps, laps;
    bool armed = false;

    //------Params-------//
    float m, Xu, Yv, Nr, xg, Iz, Ts;  

    Matrix <float, 3,3> M, M_1;
    Matrix <float, 3,3> Apsi; 
    Matrix <float, 3,3> Bpsi; 
    Matrix <float, 1,3> Cpsi; 
    Matrix <float, 6,6> Ap;
    Matrix <float, 6,3> Bp;
    Matrix <float, 2,6> Cp;  
    Matrix <float, 6,6> Tp;  
    Matrix <float, 2,2> R2T;
    Matrix <float, 3,1> tao;
    Matrix <float, 2,1> Yp; 
    Matrix <float, 6,1> Xp_hat; 
    Matrix <float, 6,1> Xp_hat_dot; 
    Matrix <float, 6,1> Xp_hat_ant; 
    Matrix <float, 3,1> Xpsi_hat; 
    Matrix <float, 3,1> Xpsi_hat_dot; 
    Matrix <float, 3,1> Xpsi_hat_ant; 

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscriber_gps_global;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_compass;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverZonoNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}