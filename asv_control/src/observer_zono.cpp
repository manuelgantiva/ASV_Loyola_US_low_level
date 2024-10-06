#include "rclcpp/rclcpp.hpp"
#include "asv_library/Zonotopo.h"
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
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

using std::placeholders::_1;

class ObserverZonoNode : public rclcpp::Node
{
public:
    ObserverZonoNode() : Node("observer_zono")
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

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        this-> declare_parameter("Max_n_r", 0.01);
        this-> declare_parameter("Max_n_p", 0.02);
        this-> declare_parameter("Max_w_r", 0.01);
        this-> declare_parameter("Max_w_p", 0.02);

        this-> declare_parameter("q", 300);

        this-> declare_parameter("Wpsi_di", std::vector<float>{1.0, 1.0, 1.0});
        this-> declare_parameter("Wp_di", std::vector<float>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        

        my_id = std::to_string(this->get_parameter("my_id").as_int());
        Ts = this->get_parameter("Ts").as_double();
        t_s = Ts/1000; // En segundos

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

        Dz_up  = this->get_parameter("Dz_up").as_double();
        Dz_down = this->get_parameter("Dz_down").as_double();

        Max_n_r = this->get_parameter("Max_n_r").as_double(); // pequeños Metodo 2
        Max_n_p = this->get_parameter("Max_n_p").as_double();
        Max_w_r = this->get_parameter("Max_w_r").as_double();
        Max_w_p = this->get_parameter("Max_w_p").as_double();

        q = this->get_parameter("q").as_int();

        std::vector<double> Wpsi_di = this->get_parameter("Wpsi_di").as_double_array();
        std::vector<double> Wp_di = this->get_parameter("Wp_di").as_double_array();

        Apsi << 1.0, t_s, 0.0,
                0.0, 1.0, t_s,
                0.0, 0.0, 1.0; 

        Cpsi << 1.0, 0.0, 0.0; 

        Bwpsi << 0.0,
                 0.0,
                 t_s; 
        
        Ap << 1.0, 0.0, t_s, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, t_s, 0.0, 0.0,
              0.0, 0.0, 1.0, 0.0, t_s, 0.0,
              0.0, 0.0, 0.0, 1.0, 0.0, t_s,
              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
              0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        Cp << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
              0.0, 1.0, 0.0, 0.0, 0.0, 0.0;

        Bwp << 0.0, 0.0,
               0.0, 0.0,
               0.0, 0.0,
               0.0, 0.0,
               t_s, 0.0,
               0.0, t_s;

        IGpsi << 0.0,
                0.0,
                0.0;

        IGp << 0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0;

        // Condiciones iniciales para los conjuntos
        Zpsi_prior= Zonotopo(VectorXd::Zero(3), MatrixXd::Identity(3,3));
        Zp_prior = Zonotopo(VectorXd::Zero(6), MatrixXd::Identity(6,6));

        Rpsi <<  Max_n_r;
        Rp << Max_n_p, 0.0,
              0.0, Max_n_p;
        
        Qpsi << Max_w_r;
        Qp << Max_w_p, 0.0,
              0.0, Max_w_p;
 
        Wpsi << Wpsi_di[0], 0, 0,
                0, Wpsi_di[1], 0,
                0, 0, Wpsi_di[2];

        Wp << Wp_di[0], 0, 0, 0, 0, 0,
              0, Wp_di[1], 0, 0, 0, 0,
              0, 0, Wp_di[2], 0, 0, 0,
              0, 0, 0, Wp_di[3], 0, 0,
              0, 0, 0, 0, Wp_di[4], 0,
              0, 0, 0, 0, 0, Wp_di[5];

        R2T.setZero();
        Yp.setZero();
    
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts)),
                                          std::bind(&ObserverZonoNode::calculateState, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObserverZonoNode::param_callback, this, _1));

        subscriber_gps_local= this-> create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose",
                rclcpp::SensorDataQoS(), std::bind(&ObserverZonoNode::callbackGpsLocalData, this, std::placeholders::_1), options_sensors_);
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/mavros/rc/out",1,
                std::bind(&ObserverZonoNode::callbackRcoutData, this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&ObserverZonoNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer_zono",
                rclcpp::SensorDataQoS());
        publisher_state_min = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer_zono_min",
                rclcpp::SensorDataQoS());
        publisher_state_max = this-> create_publisher<asv_interfaces::msg::StateObserver>("/control/state_observer_zono_max",
                rclcpp::SensorDataQoS());
        publisher_obs = this-> create_publisher<geometry_msgs::msg::PoseStamped>("pose_zono",
                rclcpp::SensorDataQoS());
        
                                        
        RCLCPP_INFO(this->get_logger(), "Observer Zonotopos Node has been started.");
    }

private:
    void calculateState()
    {
        auto msg = asv_interfaces::msg::StateObserver();
        if(armed==false){ 
            R2T.setZero();
            Yp.setZero();
            Zpsi_prior= Zonotopo(VectorXd::Zero(3), MatrixXd::Identity(3,3));
            Zp_prior = Zonotopo(VectorXd::Zero(6), MatrixXd::Identity(6,6));
            {
                std::lock_guard<std::mutex> lock(mutex_);
                Yp.setZero();
                delta_diff=0;
                delta_mean=0;
                beta=0;
                psi = 0.0;
            }
        }else{
            if(count > 5){
                //ssauto start = std::chrono::high_resolution_clock::now();
                Vector <double, 2> Yp_i;
                Vector <double, 1> Ypsi_i;
                float delta_diff_i;
                float delta_mean_i;
                int beta_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    Ypsi_i << psi;
                    Yp_i = Yp;
                    delta_diff_i = delta_diff;
                    delta_mean_i = delta_mean;
                    beta_i=beta;
                }

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

                IGp(2,0) = IGp(2,0)*0.1;
                IGp(3,0) = IGp(3,0)*0.1;
                IGpsi(1,0) = IGpsi(1,0)*0.1;

                if(count==6){
                    Eigen::VectorXd cr0(3);
                    cr0 << Ypsi_i(0), 0.0, 0.0;  
                    Eigen::VectorXd cp0(6);
                    cp0 << Yp_i(0), Yp_i(1), 0.0, 0.0, 0.0, 0.0;  
                    Zpsi_prior = Zonotopo(cr0, Eigen::MatrixXd::Identity(3, 3));  // Amplia Zonotopo si no estoy seguro
                    Zp_prior = Zonotopo(cp0, Eigen::MatrixXd::Identity(6, 6));  // Amplia Zonotopo si no estoy seguro
                    count=count+1;
                }

                // Llamar al método de filtrado
                Zpsi_next = Zonotopo::filteringPsi(Zpsi_prior, Ypsi_i, Cpsi, Rpsi, Eigen::MatrixXd::Identity(3, 3));

                //Calcular bandas psi
                MatrixXd br = rs_z(Zpsi_next);
                int nr = br.rows();
                std::vector<double> min_psi(nr), max_psi(nr);
                for (int oo = 0; oo < nr; ++oo) {
                    min_psi[oo] = br(oo, 0);  // Primera columna de br (mínimos)
                    max_psi[oo] = br(oo, 1);  // Segunda columna de br (máximos)
                }
    
                Zpsi_next.reduccionOrden(q,Wpsi);

                Zp_next = Zonotopo::filteringP(Zp_prior, Yp_i, Cp, Rp, Eigen::MatrixXd::Identity(6, 6));
                //Calcular bandas
                MatrixXd bp = rs_z(Zp_next);
                int np = bp.rows();             
                std::vector<double> min_p(np), max_p(np);
                for (int oo = 0; oo < np; ++oo) {
                    min_p[oo] = bp(oo, 0);  // Primera columna de bp (mínimos)
                    max_p[oo] = bp(oo, 1);  // Segunda columna de bp (máximos)
                }

                Zp_next.reduccionOrden(2*q,Wp);

                VectorXd qr = Apsi * Zpsi_next.c + IGpsi;
                MatrixXd Hr1 = Apsi * Zpsi_next.H;
                MatrixXd Hr2 = Bwpsi * Qpsi;
                MatrixXd Hr(Hr1.rows(), Hr1.cols() + Hr2.cols());
                Hr << Hr1, Hr2;
                Zpsi_prior = Zonotopo(qr, Hr);

                Zp_prior = Zonotopo::prediction_Y(Ap,Zp_next,Ypsi_i(0)-Max_n_r,Ypsi_i(0)+Max_n_r,Bwp,Qp,IGp);

                msg.header.stamp = this->now();
                msg.header.frame_id = my_id; 

                msg.point.x=Zp_next.c(0);
                msg.point.y=Zp_next.c(1);
                msg.point.z=Zpsi_next.c(0);
                msg.velocity.x=Zp_next.c(2);
                msg.velocity.y=Zp_next.c(3);
                msg.velocity.z=Zpsi_next.c(1);
                msg.disturbances.x=Zp_next.c(4);
                msg.disturbances.y=Zp_next.c(5);
                msg.disturbances.z=Zpsi_next.c(2);
                publisher_state->publish(msg);

                msg.point.x=min_p[0];
                msg.point.y=min_p[1];
                msg.point.z=min_psi[0];
                msg.velocity.x=min_p[2];
                msg.velocity.y=min_p[3];
                msg.velocity.z=min_psi[1];
                msg.disturbances.x=min_p[4];
                msg.disturbances.y=min_p[5];
                msg.disturbances.z=min_psi[2];
                publisher_state_min->publish(msg);

                msg.point.x=max_p[0];
                msg.point.y=max_p[1];
                msg.point.z=max_psi[0];
                msg.velocity.x=max_p[2];
                msg.velocity.y=max_p[3];
                msg.velocity.z=max_psi[1];
                msg.disturbances.x=max_p[4];
                msg.disturbances.y=max_p[5];
                msg.disturbances.z=max_psi[2];
                publisher_state_max->publish(msg);

                auto msg_obs = geometry_msgs::msg::PoseStamped();

                msg_obs.header.stamp = this->now();
                msg_obs.header.frame_id = "map_ned";
                msg_obs.pose.position.x= Zp_next.c(0);
                msg_obs.pose.position.y= Zp_next.c(1);
                msg_obs.pose.position.z= 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, Zpsi_next.c(0));
                msg_obs.pose.orientation.x = q.x();
                msg_obs.pose.orientation.y = q.y();
                msg_obs.pose.orientation.z = q.z();
                msg_obs.pose.orientation.w = q.w();
                publisher_obs->publish(msg_obs); 

                //auto end = std::chrono::high_resolution_clock::now();
                //std::chrono::duration<double> elapsed = end - start;
                //double miliseconds = elapsed.count()*1000;
                //RCLCPP_INFO(this->get_logger(), "Exec time: %f", miliseconds);
            }else{
                count=count+1;
            }
        }
    }

    void callbackGpsLocalData(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if(armed==true){
            float y = msg->pose.position.x;
            float x = msg->pose.position.y;
            float psi_rad = quat2EulerAngles_XYZ(msg->pose.orientation.w, msg->pose.orientation.x,
                                                msg->pose.orientation.y, msg->pose.orientation.z);
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
            //RCLCPP_INFO(this->get_logger(), "n is: %d", int(laps));
            //RCLCPP_INFO(this->get_logger(), "Heading is: %f", psi);
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
                beta=beta_a;
            }
            // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
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

    // Función que calcula los mínimos y máximos de las variables de estado
    MatrixXd rs_z(const Zonotopo& z) {
        VectorXd q = z.c;     // Vector central
        MatrixXd H = z.H;     // Matriz generadora

        int fil = H.rows();   // Número de filas de H
        int col = H.cols();   // Número de columnas de H

        // Inicialización de los vectores de mínimos y máximos
        VectorXd mini(fil);
        VectorXd maxi(fil);

        for (int i = 0; i < fil; ++i) {
            double aux = 0;
            for (int j = 0; j < col; ++j) {
                aux += std::abs(H(i, j));  // Sumar el valor absoluto de cada elemento
            }
            mini(i) = q(i) - aux;  // Cálculo del mínimo
            maxi(i) = q(i) + aux;  // Cálculo del máximo
        }

        // Concatenar mínimos y máximos en una sola matriz
        MatrixXd rs(fil, 2);
        rs << mini, maxi;
        return rs;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "Max_n_p"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Max_n_p = param.as_double();
                    Rp << Max_n_p, 0.0,
                        0.0, Max_n_p;
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Max_n_r"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Max_n_r = param.as_double();
                    Rpsi <<  Max_n_r;        
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Max_w_p"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Max_w_p = param.as_double();
                    Qp << Max_w_p, 0.0,
                        0.0, Max_w_p;
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Max_w_r"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Max_w_r = param.as_double();      
                    Qpsi << Max_w_r;
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "q"){
                if(param.as_int() >= 0 and param.as_int() < 500){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    q = this->get_parameter("q").as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Wpsi_di"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Wpsi_di = param.as_double_array();
                    Wpsi << Wpsi_di[0], 0, 0,
                            0, Wpsi_di[1], 0,
                            0, 0, Wpsi_di[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Wp_di"){
                if(param.as_double_array().size() == 6){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Wp_di = param.as_double_array();
                    Wp << Wp_di[0], 0, 0, 0, 0, 0,
                          0, Wp_di[1], 0, 0, 0, 0,
                          0, 0, Wp_di[2], 0, 0, 0,
                          0, 0, 0, Wp_di[3], 0, 0,
                          0, 0, 0, 0, Wp_di[4], 0,
                          0, 0, 0, 0, 0, Wp_di[5];
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

    float psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;

 //------Params-------//
    std::string my_id;
    float Ts, t_s, Xu6, Xu7, Xv10, Xv11, Xv12, Xv13, Xr10, Xr11 ,Xr12, Xr13;  
    float Dz_up, Dz_down;  

    float Max_w_r, Max_w_p, Max_n_p, Max_n_r;

    float delta_diff;
    float delta_mean;
    int beta, count=0, q;  

    Matrix <double, 3,1> IGpsi; 
    Matrix <double, 6,1> IGp;
    Matrix <double, 2,2> R2T;
    Vector <double, 2> Yp; 
    Matrix <double, 3,3> Apsi;
    Matrix <double, 1,3> Cpsi;
    Matrix <double, 3,1> Bwpsi;
    Matrix <double, 6,6> Ap;
    Matrix <double, 2,6> Cp;
    Matrix <double, 6,2> Bwp;
    Matrix <double, 1,1> Rpsi;
    Matrix <double, 2,2> Rp;
    Matrix <double, 1,1> Qpsi;
    Matrix <double, 2,2> Qp;
    Matrix <double, 3,3> Wpsi;
    Matrix <double, 6,6> Wp;
    
    Zonotopo Zp_prior, Zpsi_prior, Zp_next, Zpsi_next;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_obs;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_min;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_max;
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
    auto node = std::make_shared<ObserverZonoNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}