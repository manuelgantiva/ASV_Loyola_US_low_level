#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "std_msgs/msg/float32_multi_array.hpp"

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
        this-> declare_parameter("my_id", "ASV0");
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        
        this-> declare_parameter("Xu", std::vector<double>{1.0, 1.0});
        this-> declare_parameter("Xv", std::vector<double>{1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xr", std::vector<double>{1.0, 1.0, 1.0, 1.0});

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        this-> declare_parameter("PpWp_c1", std::vector<float>{6.567173587771372, 0.0, 12.050184632655890, 0.0, 6.992727395821864, 0.0});
        this-> declare_parameter("PpWp_c2", std::vector<float>{0.0, 6.567173587771372, 0.0, 12.050184632655867, 0.0, 6.992727395821881});
        this-> declare_parameter("Lpsi", std::vector<float>{10.370372264461590, 42.762463429973230, 76.008320148412050});

        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        
        Xu = this->get_parameter("Xu").as_double_array();
        Xv = this->get_parameter("Xv").as_double_array();
        Xr = this->get_parameter("Xr").as_double_array();

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

        subscription_data = this->create_subscription<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/data_sensors", rclcpp::SensorDataQoS(),
            std::bind(&ObserverLiuNode::calculateState, this, std::placeholders::_1),options_sensors_);

        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&ObserverLiuNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/" + my_id + "/observer/state_observer_liu",
                rclcpp::SensorDataQoS());

        publisher_obs = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/observer/pose_liu",
                rclcpp::SensorDataQoS());
                                        
        RCLCPP_INFO(this->get_logger(), "Observer Liu Node in %s has been started.", my_id.c_str());
    	
    }

private:
    void calculateState(const std_msgs::msg::Float32MultiArray::SharedPtr msg_data)
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
        }else{
            if(count > 6){
                //auto start = std::chrono::high_resolution_clock::now();
                float psi_i;
                Matrix <float, 2,1> Yp_i;
                float delta_diff_i;
                float delta_mean_i;
                int beta_i;
                
                Yp_i << static_cast<double>(msg_data->data[0]),
                        static_cast<double>(msg_data->data[1]);
                psi_i = static_cast<double>(msg_data->data[2]);

                delta_diff_i = msg_data->data[4];
                delta_mean_i = msg_data->data[5];
                beta_i=static_cast<int>(msg_data->data[6]);

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
                
                IGp(2,0) = ( (Xu[1]*sum_1)+(Xu[2]*delta_mean_i) )*0.1;
                IGp(3,0) = ( (Xv[1]*sum_1*(1-beta_i)*sig)+(Xv[2]*delta_mean_i*delta_diff_i)+(Xv[3]*delta_mean_i*(1-beta_i)*sig)+(Xv[4]*delta_diff_i/2.0) )*0.1;
                IGpsi(1,0) = ( (Xr[1]*sum_1*(1-beta_i)*sig)+(Xr[2]*delta_mean_i*delta_diff_i)+(Xr[3]*delta_mean_i*(1-beta_i)*sig)+(Xr[4]*delta_diff_i/2.0) )*0.1;

                Lp=Tp.inverse()*PpWp*R2T; 

                if(count==7){
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

    bool armed = false, armed_act = false;

    //------Params-------//
    std::string my_id;
    float Ts;  
    std::vector<double> Xu, Xv, Xr;

    int  count=0;

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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_data;
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