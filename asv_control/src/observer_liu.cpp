#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <cmath>
#include <thread>
#include <Eigen/Dense>

using namespace Eigen;

using std::placeholders::_1;

class ObserverLiuNode : public rclcpp::Node
{
public:
    ObserverLiuNode() : Node("observer_liu")
    {
        this-> declare_parameter("my_id", 0);
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        
        this-> declare_parameter("Xu6", -0.003148);
        this-> declare_parameter("Xu7", 0.0810014);
        this-> declare_parameter("Xv10", -0.00394830);
        this-> declare_parameter("Xv11", -0.0183636);
        this-> declare_parameter("Xv12", 0.0);
        this-> declare_parameter("Xv13", -0.0200932);
        this-> declare_parameter("Xr10", -0.0129643);
        this-> declare_parameter("Xr11", -0.0110386);
        this-> declare_parameter("Xr12", 0.0);
        this-> declare_parameter("Xr13", 0.1717909);

        this-> declare_parameter("PpWp_c1", std::vector<float>{6.567173587771372, 0.0, 12.050184632655890, 0.0, 6.992727395821864, 0.0});
        this-> declare_parameter("PpWp_c2", std::vector<float>{0.0, 6.567173587771372, 0.0, 12.050184632655867, 0.0, 6.992727395821881});
        this-> declare_parameter("Lpsi", std::vector<float>{10.370372264461590, 42.762463429973230, 76.008320148412050});

        my_id = std::to_string(this->get_parameter("my_id").as_int());
        Ts = this->get_parameter("Ts").as_double();
        
        Xu6 = this->get_parameter("Xu6").as_double();
        Xu7 = this->get_parameter("Xu7").as_double();
        Xv10 = this->get_parameter("Xv10").as_double();
        Xv11 = this->get_parameter("Xv11").as_double();
        Xv12 = this->get_parameter("Xv12").as_double();
        Xv13 = this->get_parameter("Xv13").as_double();
        Xr10 = this->get_parameter("Xr10").as_double();
        Xr11 = this->get_parameter("Xr11").as_double();
        Xr12 = this->get_parameter("Xr12").as_double();
        Xr13 = this->get_parameter("Xr13").as_double();

        std::vector<double> Lpsi_par= this->get_parameter("Lpsi").as_double_array();
        std::vector<double> PpWpc1_par = this->get_parameter("PpWp_c1").as_double_array();
        std::vector<double> PpWpc2_par = this->get_parameter("PpWp_c2").as_double_array();

        Apsi << 0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                0.0, 0.0, 0.0;     

        IGpsi << 0.0,
                0.0,
                0.0;

        Cpsi << 1.0, 0.0, 0.0;

        Ap << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

        IGp << 0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0;

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
        Lp.setZero();
        Xp_hat.setZero();
        Xp_hat_dot.setZero(); 
        Xp_hat_ant.setZero(); 
        Xpsi_hat.setZero(); 
        Xpsi_hat_dot.setZero(); 
        Xpsi_hat_ant.setZero(); 

        PpWp << PpWpc1_par[0], PpWpc2_par[0],
                PpWpc1_par[1], PpWpc2_par[1],
                PpWpc1_par[2], PpWpc2_par[2],
                PpWpc1_par[3], PpWpc2_par[3],
                PpWpc1_par[4], PpWpc2_par[4],
                PpWpc1_par[5], PpWpc2_par[5];

        Lpsi << Lpsi_par[0],
                Lpsi_par[1],
                Lpsi_par[2];

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObserverLiuNode::param_callback, this, _1));

        subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
                rclcpp::SensorDataQoS(), std::bind(&ObserverLiuNode::callbackGpsLocalData, this, std::placeholders::_1), options_sensors_);
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/mavros/rc/out",10,
                std::bind(&ObserverLiuNode::callbackRcoutData, this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ObserverLiuNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer_liu",
                rclcpp::SensorDataQoS());
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts)),
                                          std::bind(&ObserverLiuNode::calculateState, this), cb_group_obs_);
        publisher_obs = this-> create_publisher<geometry_msgs::msg::PoseStamped>("pose_liu",
                rclcpp::SensorDataQoS());
                                        
        RCLCPP_INFO(this->get_logger(), "Observer Liu Node has been started.");
    	
    }

private:
    void calculateState()
    {
        auto msg = asv_interfaces::msg::StateObserver();
        if(armed==false){
            count=0;
            Xp_hat.setZero();
            Xp_hat_dot.setZero(); 
            Xp_hat_ant.setZero(); 
            Xpsi_hat.setZero(); 
            Xpsi_hat_dot.setZero(); 
            Xpsi_hat_ant.setZero(); 
            R2T.setZero();
            Lp.setZero();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                Yp.setZero();
                delta_diff=0;
                delta_mean=0;
                beta=0;
            }
        }else{
            if(count > 5){
                //auto start = std::chrono::high_resolution_clock::now();
                float psi_i;
                Matrix <float, 2,1> Yp_i;
                float delta_diff_i;
                float delta_mean_i;
                int beta_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    psi_i = psi;
                    Yp_i = Yp;
                    delta_diff_i = delta_diff;
                    delta_mean_i = delta_mean;
                    beta_i=beta;
                }
                float cospsi= cos(psi_i);
                float senpsi= sin(psi_i);

                R2T << cospsi, senpsi,
                    -senpsi, cospsi;

                Tp(0,0)=cospsi;
                Tp(0,1)=senpsi;
                Tp(1,0)=-senpsi;
                Tp(1,1)=cospsi;

                Ap(0,2)=cospsi;
                Ap(0,3)=-senpsi;
                Ap(1,2)=senpsi;
                Ap(1,3)=cospsi;

                float delta_mean_i_2 = delta_mean_i*delta_mean_i;
                float delta_diff_i_2 = delta_diff_i*delta_diff_i;
                float sum_1 = (delta_mean_i_2+(delta_diff_i_2/4.0));
                float sig;
                if(delta_diff_i>=0){
                    sig=1;
                }else{
                    sig=-1;
                }
                
                IGp(2,0) = (Xu6*sum_1)+(Xu7*delta_mean_i);
                IGp(3,0) = (Xv10*sum_1*(1-beta_i)*sig)+(Xv11*delta_mean_i*delta_diff_i)+(Xv12*delta_mean_i*(1-beta_i)*sig)+(Xv13*delta_diff_i/2.0);
                IGpsi(1,0)=(Xr10*sum_1*(1-beta_i)*sig)+(Xr11*delta_mean_i*delta_diff_i)+(Xr12*delta_mean_i*(1-beta_i)*sig)+(Xr13*delta_diff_i/2.0);

                Lp=Tp.inverse()*PpWp*R2T; 

                if(count==6){
                    Xp_hat_ant << Yp_i(0,0),
                                Yp_i(1,0),
                                0.0,
                                0.0,
                                0.0,
                                0.0;
                    Xp_hat_dot.setZero(); 

                    Xpsi_hat_ant << psi_i,
                                0.0,
                                0.0;
                    Xpsi_hat_dot.setZero(); 
                    count=count+1;
                }

                Xp_hat = Xp_hat_ant + Xp_hat_dot*(Ts/1000.0);
                Xp_hat_ant = Xp_hat;
                Xp_hat_dot = Ap*Xp_hat + IGp + Lp*(Yp_i - Cp*Xp_hat);

                Xpsi_hat = Xpsi_hat_ant + Xpsi_hat_dot*(Ts/1000.0);
                Xpsi_hat_ant = Xpsi_hat;
                Xpsi_hat_dot = Apsi*Xpsi_hat + IGpsi + Lpsi*(psi_i - Cpsi*Xpsi_hat);

                msg.header.stamp = this->now();
                msg.header.frame_id = my_id; 

                msg.point.x=Xp_hat(0,0);
                msg.point.y=Xp_hat(1,0);
                msg.point.z=Xpsi_hat(0,0);
                msg.velocity.x=Xp_hat(2,0);
                msg.velocity.y=Xp_hat(3,0);
                msg.velocity.z=Xpsi_hat(1,0);
                msg.disturbances.x=Xp_hat(4,0);
                msg.disturbances.y=Xp_hat(5,0);
                msg.disturbances.z=Xpsi_hat(2,0);

                publisher_state->publish(msg);

                auto msg_obs = geometry_msgs::msg::PoseStamped();

                msg_obs.header.stamp = this->now();
                msg_obs.header.frame_id = "map_ned";
                msg_obs.pose.position.x= Xp_hat(0,0);
                msg_obs.pose.position.y= Xp_hat(1,0);
                msg_obs.pose.position.z= 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, Xpsi_hat(0,0));
                msg_obs.pose.orientation.x = q.x();
                msg_obs.pose.orientation.y = q.y();
                msg_obs.pose.orientation.z = q.z();
                msg_obs.pose.orientation.w = q.w();
                publisher_obs->publish(msg_obs); 

                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start;
                // double miliseconds = elapsed.count()*1000;
                // RCLCPP_INFO(this->get_logger(), "Exec time: %f", miliseconds);
            }else{
                count=count+1;
            }
        }        
    }

    void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(armed==true){
            if (status_gps!=-1){
                float y = msg->pose.position.x;
                float x = msg->pose.position.y;
                float psi_rad = quat2EulerAngles_XYZ(msg->pose.orientation.w, msg->pose.orientation.x,
                                                    msg->pose.orientation.y, msg->pose.orientation.z);
                psi_rad=-psi_rad+1.5708;
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
                        psi = psi_rad;
                        Yp << x,
                            y;
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
                        psi = psi_act + 2*M_PI*laps;
                        Yp << x,
                            y;
                    }
                    psi_ant=psi_act;
                }
                //RCLCPP_INFO(this->get_logger(), "n is: %d", int(laps));
                //RCLCPP_INFO(this->get_logger(), "Heading is: %f", psi);
                //RCLCPP_INFO(this->get_logger(), "gps x is: %f, y is: %f", x, y);
            }
            // RCLCPP_INFO(this->get_logger(), "Satus gps is: %d", int(status_gps));
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
        const double psi = atan2(m12, m11);
        return static_cast<float>(psi);
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
                beta=beta_a;
            }
            // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
        }
    }

    float deleteDeadZone(float delta)
    {
        if(delta>-0.08 && delta<0.075){
            delta = 0;
        }else if(delta <= -0.08){
            delta = delta+0.08;
        }else if(delta >= 0.075){
            delta = delta-0.075;
        }
        return delta;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }
    
    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "Lpsi"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Lpsi_par = param.as_double_array();
                    Lpsi << Lpsi_par[0],
                            Lpsi_par[1],
                            Lpsi_par[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "array out of range";
                    return result;
                }
            }
            if (param.get_name() == "PpWp_c1"){
                if(param.as_double_array().size() == 6){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> PpWpc1_par = param.as_double_array();
                    PpWp(0,0)=PpWpc1_par[0];
                    PpWp(1,0)=PpWpc1_par[1];
                    PpWp(2,0)=PpWpc1_par[2];
                    PpWp(3,0)=PpWpc1_par[3];
                    PpWp(4,0)=PpWpc1_par[4];
                    PpWp(5,0)=PpWpc1_par[5];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 6");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "PpWp_c2"){
                if(param.as_double_array().size() == 6){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> PpWpc2_par = param.as_double_array();
                    PpWp(0,1)=PpWpc2_par[0];
                    PpWp(1,1)=PpWpc2_par[1];
                    PpWp(2,1)=PpWpc2_par[2];
                    PpWp(3,1)=PpWpc2_par[3];
                    PpWp(4,1)=PpWpc2_par[4];
                    PpWp(5,1)=PpWpc2_par[5];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 6");
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


    float yaw, lat, lon, alt, psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;

    //------Params-------//
    std::string my_id;
    float Ts, Xu6, Xu7, Xv10, Xv11, Xv12, Xv13, Xr10, Xr11 ,Xr12, Xr13;  

    float delta_diff;
    float delta_mean;
    int beta, count=0;

    Matrix <float, 3,3> Apsi; 
    Matrix <float, 3,1> IGpsi; 
    Matrix <float, 1,3> Cpsi; 
    Matrix <float, 6,6> Ap;
    Matrix <float, 6,1> IGp;
    Matrix <float, 2,6> Cp;  
    Matrix <float, 6,6> Tp;  
    Matrix <float, 6,2> Lp; 
    Matrix <float, 2,2> R2T;
    Matrix <float, 6,2> PpWp; 
    Matrix <float, 3,1> Lpsi;
    Matrix <float, 3,1> tao;
    Matrix <float, 2,1> Yp; 
    Matrix <float, 6,1> Xp_hat; 
    Matrix <float, 6,1> Xp_hat_dot; 
    Matrix <float, 6,1> Xp_hat_ant; 
    Matrix <float, 3,1> Xpsi_hat; 
    Matrix <float, 3,1> Xpsi_hat_dot; 
    Matrix <float, 3,1> Xpsi_hat_ant; 

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_obs;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state;
    rclcpp::TimerBase::SharedPtr timer_;
   
    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObserverLiuNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}