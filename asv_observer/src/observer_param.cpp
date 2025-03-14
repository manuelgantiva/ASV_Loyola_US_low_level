#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "nav_msgs/msg/odometry.hpp"                //Interface gps global local data
#include "std_msgs/msg/float32_multi_array.hpp"          // Interface coeficientes polinomio
#include "std_msgs/msg/bool.hpp"                    //Interface armed data

// #include "tf2/exceptions.h"
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"

#include <cmath>
#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

using std::placeholders::_1;

class ObserverParamNode : public rclcpp::Node
{
public:
    ObserverParamNode() : Node("observer_param")
    {
        this-> declare_parameter("my_id", "ASV1");
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        // TODO poner parametros en .yaml
        this-> declare_parameter("Xu", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xv", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xr", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        this-> declare_parameter("R", 0.01);

        // variacion de parámetros
        this-> declare_parameter("max_diff_Xu", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        this-> declare_parameter("max_diff_Xv", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        this-> declare_parameter("max_diff_Xr", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        t_s = Ts/1000; // En segundos

        std::copy(Xu.begin(), Xu.end(), Xu.data());
        std::copy(Xv.begin(), Xv.end(), Xv.data());
        std::copy(Xr.begin(), Xr.end(), Xr.data());

        std::copy(max_diff_Xu.begin(), max_diff_Xu.end(), max_diff_Xu.data());
        std::copy(max_diff_Xv.begin(), max_diff_Xv.end(), max_diff_Xv.data());
        std::copy(max_diff_Xr.begin(), max_diff_Xr.end(), max_diff_Xr.data());

        /* Xu = this->get_parameter("Xu").as_double_array();
        Xv = this->get_parameter("Xv").as_double_array();
        Xr = this->get_parameter("Xr").as_double_array();*/
        Dz_up  = this->get_parameter("Dz_up").as_double();
        Dz_down = this->get_parameter("Dz_down").as_double();

        R_value = this->get_parameter("R").as_double();
        
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts)),
                                          std::bind(&ObserverParamNode::calculateState, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObserverParamNode::param_callback, this, _1));

        // suscribir a IMU 
        subscriber_imu = this-> create_subscription<sensor_msgs::msg::Imu>("/" + my_id + "/mavros/imu/data",
                rclcpp::SensorDataQoS(), std::bind(&ObserverParamNode::callbackImuData,
                this, std::placeholders::_1), options_sensors_);
        subscriber_gps_local= this-> create_subscription<nav_msgs::msg::Odometry>("/" + my_id + "/mavros/global_position/local",
                rclcpp::SensorDataQoS(), std::bind(&ObserverParamNode::callbackGpsLocalData, this, std::placeholders::_1), options_sensors_);
        subscriber_rcout = this-> create_subscription<mavros_msgs::msg::RCOut>("/" + my_id + "/mavros/rc/out",1,
                std::bind(&ObserverParamNode::callbackRcoutData, this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&ObserverParamNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        publisher_param = this-> create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/observer_param",
                    rclcpp::SensorDataQoS());
        publisher_kalmangains = this-> create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/kalman_gains",
                    rclcpp::SensorDataQoS());
        publisher_velocities = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/observer/velocities",
                    rclcpp::SensorDataQoS());
        publisher_setparam = this-> create_publisher<std_msgs::msg::Bool>("/" + my_id + "/observer/set_param",
                    1);
        
                                            
        RCLCPP_INFO(this->get_logger(), "Observer Parameters Node in %s has been started.", my_id.c_str());
    }

private:
    void calculateState()
    {
        auto msg = asv_interfaces::msg::StateObserver();
        if(armed==false){ 
            count=0;
            count_set=0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                bufferX.clear();
                bufferY.clear();
                bufferPsi.clear();
                bufferR.clear();
                bufferdelta_diff.clear();
                bufferdelta_mean.clear();
                // Inicializar ganancias 
                L_u = Eigen::MatrixXd::Identity(7, 1);
                L_v = Eigen::MatrixXd::Identity(13, 1);
                L_r = Eigen::MatrixXd::Identity(13, 1);
                // Matriz de covarianza de la predicción (P_bar)
                P_bar_u = Eigen::MatrixXd::Identity(7, 7);
                P_bar_v = Eigen::MatrixXd::Identity(13, 13);
                P_bar_r = Eigen::MatrixXd::Identity(13, 13);
                // Matriz de covarianza del ruido de medida (R)
                R = Eigen::Matrix<double, 1, 1> {R_value};
            }
        }else{
            if(count > n_SG+6){
                double delta_mean_km1, delta_diff_km1, u_km1, v_km1, r_km1, u_k, v_k, r_k, d;
                Eigen::Matrix<double,1 ,1> bu, bv, br;
                int beta;

                if(count==n_SG+2){
                    count=count+1;
                }

                obtainVelocities();
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    int km1 = n_SG/2-1;
                    delta_diff_km1 = bufferdelta_diff[km1];
                    delta_mean_km1 = bufferdelta_mean[km1];
                    beta = bufferBeta[km1];
                    u_km1 = prev_u;
                    v_km1 = prev_v;
                    r_km1 = bufferR[km1];
                    u_k = u;
                    v_k = v;
                    r_k = bufferR[km1+1];
                }

                int sig;
                if(delta_diff_km1==0){
                    sig=1;
                }else{
                    sig=-1;
                }

                d = delta_mean_km1*delta_mean_km1 + 0.25*delta_diff_km1*delta_diff_km1;
                //RCLCPP_INFO(this->get_logger(), "Antes de definir matrices A");
                Eigen::Matrix<double, 1, 7> Au = {u_km1*abs(u_km1), v_km1*r_km1, r_km1*r_km1, u_km1, d, delta_mean_km1, 1};
                Eigen::Matrix<double, 1, 13> Av = {v_km1*abs(v_km1), v_km1*abs(r_km1), r_km1*abs(v_km1), r_km1*abs(r_km1), u_km1*v_km1, u_km1*r_km1, v_km1, r_km1, sig*(1-beta)*d, delta_diff_km1*delta_mean_km1, sig*(1-beta)*delta_mean_km1, delta_diff_km1/2, 1};
                Eigen::Matrix<double, 1, 13> Ar = Av;

                bu << u_k - u_km1;
                bv << v_k - v_km1;
                br << r_k - r_km1;

                // Filtro de Kalman para obtener Xu, Xv, Xr
                //RCLCPP_INFO(this->get_logger(), "Despues de definir matrices A, antes de X");
                Eigen::Matrix<double, 7, 1> Xu_km1 = {Xu[0], Xu[1], Xu[2], Xu[3], Xu[4], Xu[5], Xu[6]};
                Eigen::Matrix<double, 13, 1> Xv_km1 = {Xv[0], Xv[1], Xv[2], Xv[3], Xv[4], Xv[5], Xv[6], Xv[7], Xv[8], Xv[9], Xv[10], Xv[11], Xv[12]};
                Eigen::Matrix<double, 13, 1> Xr_km1 = {Xr[0], Xr[1], Xr[2], Xr[3], Xr[4], Xr[5], Xr[6], Xr[7], Xr[8], Xr[9], Xr[10], Xr[11], Xr[12]};
                
                // Etapa de Filtrado
                //RCLCPP_INFO(this->get_logger(), "Despues de definir matrices X");
                Xu = Xu_km1 + L_u * (bu - Au*Xu_km1);
                Xv = Xv_km1 + L_v * (bv - Av*Xv_km1);
                Xr = Xr_km1 + L_r * (br - Ar*Xr_km1);
                // RCLCPP_INFO(this->get_logger(),  "Valores actualizados de X : %f, %f, %f.", Xu(0,0), Xv(0,0), Xr(0,0));

                // Actualizar ganancias de Kalman
                L_u = P_bar_u * Au.transpose() * (Au * P_bar_u * Au.transpose() + R).inverse();
                L_v = P_bar_v * Av.transpose() * (Av * P_bar_v * Av.transpose() + R).inverse();
                L_r = P_bar_r * Ar.transpose() * (Ar * P_bar_r * Ar.transpose() + R).inverse();
                //RCLCPP_INFO(this->get_logger(), "Valores actualizados de L : %f, %f, %f.", L_u(0,0), L_v(0,0), L_r(0,0));

                // Actualizar matriz de covarianza (P_bar)
                P_bar_u = (Eigen::MatrixXd::Identity(7, 7) - L_u * Au) * P_bar_u;
                P_bar_v = (Eigen::MatrixXd::Identity(13, 13) - L_v * Av) * P_bar_v;
                P_bar_r = (Eigen::MatrixXd::Identity(13, 13) - L_r * Ar) * P_bar_r;


                // Publicar los valores de Xu, Xv, Xr
                std_msgs::msg::Float32MultiArray msg;

                // Configuramos el layout para un arreglo unidimensional
                msg.layout.dim.resize(1);
                msg.layout.dim[0].label = "state_estimation";
                msg.layout.dim[0].size = 7 + 13 + 13;    // Total de elementos
                msg.layout.dim[0].stride = 7 + 13 + 13;  // En este caso, stride = tamaño total

                // Limpiamos el vector de datos
                msg.data.clear();

                // Agregamos los elementos de Xu (suponiendo que Xu es accesible como un array o vector)
                for (size_t i = 0; i < 7; ++i) {
                    msg.data.push_back(static_cast<float>(Xu[i]));
                }

                // Agregamos los elementos de Xv
                for (size_t i = 0; i < 13; ++i) {
                    msg.data.push_back(static_cast<float>(Xv[i]));
                }

                // Agregamos los elementos de Xr
                for (size_t i = 0; i < 13; ++i) {
                    msg.data.push_back(static_cast<float>(Xr[i]));
                }

                // Publicamos el mensaje
                publisher_param->publish(msg);

                // Publicar los valores de las ganancias de Kalman
                std_msgs::msg::Float32MultiArray msg_kg;
                
                // Configuramos el layout para un arreglo unidimensional
                msg_kg.layout.dim.resize(1);
                msg_kg.layout.dim[0].label = "kalman_gains";
                msg_kg.layout.dim[0].size = 7 + 13 + 13;    // Total de elementos
                msg_kg.layout.dim[0].stride = 7 + 13 + 13;  // En este caso, stride = tamaño total

                // Limpiamos el vector de datos
                msg_kg.data.clear();

                // Agregamos los elementos de L_u (suponiendo que L_u es accesible como un array o vector)
                for (size_t i = 0; i < 7; ++i) {
                    msg_kg.data.push_back(static_cast<float>(L_u[i]));
                }
                
                // Agregamos los elementos de L_v
                for (size_t i = 0; i < 13; ++i) {
                    msg_kg.data.push_back(static_cast<float>(L_v[i]));
                }

                // Agregamos los elementos de L_r
                for (size_t i = 0; i < 13; ++i) {
                    msg_kg.data.push_back(static_cast<float>(L_r[i]));
                }

                // Publicamos el mensaje
                publisher_kalmangains->publish(msg_kg);

                // Publicar los valores de las velocidades surge, sway y yaw
                geometry_msgs::msg::PoseStamped msg_velocities;
                msg_velocities.header.stamp = this->now();
                msg_velocities.header.frame_id = "odom";
                msg_velocities.pose.position.x = u;
                msg_velocities.pose.position.y = v;
                msg_velocities.pose.position.z = r;
                publisher_velocities->publish(msg_velocities);


                // Publicar mensaje para setear los parámetros en el observador de estados
                count_set = count_set + 1;
                Eigen::Matrix<double, 7, 1> max_diff_Xu = {max_diff_Xu[0], max_diff_Xu[1], max_diff_Xu[2], max_diff_Xu[3], max_diff_Xu[4], max_diff_Xu[5], max_diff_Xu[6]};
                Eigen::Matrix<double, 13, 1> max_diff_Xv = {max_diff_Xv[0], max_diff_Xv[1], max_diff_Xv[2], max_diff_Xv[3], max_diff_Xv[4], max_diff_Xv[5], max_diff_Xv[6], max_diff_Xv[7], max_diff_Xv[8], max_diff_Xv[9], max_diff_Xv[10], max_diff_Xv[11], max_diff_Xv[12]};
                Eigen::Matrix<double, 13, 1> max_diff_Xr = {max_diff_Xr[0], max_diff_Xr[1], max_diff_Xr[2], max_diff_Xr[3], max_diff_Xr[4], max_diff_Xr[5], max_diff_Xr[6], max_diff_Xr[7], max_diff_Xr[8], max_diff_Xr[9], max_diff_Xr[10], max_diff_Xr[11], max_diff_Xr[12]};
                // si se ha superado el número de iteraciones
                if (count_set > 6000)
                {
                    auto msg_set = std_msgs::msg::Bool();
                    msg_set.data = true;
                    publisher_setparam->publish(msg_set);
                }
                // si la diferencia de Xu y max_diff_Xu es mayor a un umbral
                if (std::abs(Xu[0] - Xu_km1[0]) > max_diff_Xu[0] || std::abs(Xu[1] - Xu_km1[1]) > max_diff_Xu[1] || std::abs(Xu[2] - Xu_km1[2]) > max_diff_Xu[2] || std::abs(Xu[3] - Xu_km1[3]) > max_diff_Xu[3] || std::abs(Xu[4] - Xu_km1[4]) > max_diff_Xu[4] || std::abs(Xu[5] - Xu_km1[5]) > max_diff_Xu[5] || std::abs(Xu[6] - Xu_km1[6]) > max_diff_Xu[6])
                {
                    auto msg_set = std_msgs::msg::Bool();
                    msg_set.data = true;
                    publisher_setparam->publish(msg_set);
                }
                // si la diferencia de Xv y max_diff_Xv es mayor a un umbral
                if (std::abs(Xv[0] - Xv_km1[0]) > max_diff_Xv[0] || std::abs(Xv[1] - Xv_km1[1]) > max_diff_Xv[1] || std::abs(Xv[2] - Xv_km1[2]) > max_diff_Xv[2] || std::abs(Xv[3] - Xv_km1[3]) > max_diff_Xv[3] || std::abs(Xv[4] - Xv_km1[4]) > max_diff_Xv[4] || std::abs(Xv[5] - Xv_km1[5]) > max_diff_Xv[5] || std::abs(Xv[6] - Xv_km1[6]) > max_diff_Xv[6] || std::abs(Xv[7] - Xv_km1[7]) > max_diff_Xv[7] || std::abs(Xv[8] - Xv_km1[8]) > max_diff_Xv[8] || std::abs(Xv[9] - Xv_km1[9]) > max_diff_Xv[9] || std::abs(Xv[10] - Xv_km1[10]) > max_diff_Xv[10] || std::abs(Xv[11] - Xv_km1[11]) > max_diff_Xv[11] || std::abs(Xv[12] - Xv_km1[12]) > max_diff_Xv[12])
                {
                    auto msg_set = std_msgs::msg::Bool();
                    msg_set.data = true;
                    publisher_setparam->publish(msg_set);
                }
                // si la diferencia de Xr y max_diff_Xr es mayor a un umbral
                if (std::abs(Xr[0] - Xr_km1[0]) > max_diff_Xr[0] || std::abs(Xr[1] - Xr_km1[1]) > max_diff_Xr[1] || std::abs(Xr[2] - Xr_km1[2]) > max_diff_Xr[2] || std::abs(Xr[3] - Xr_km1[3]) > max_diff_Xr[3] || std::abs(Xr[4] - Xr_km1[4]) > max_diff_Xr[4] || std::abs(Xr[5] - Xr_km1[5]) > max_diff_Xr[5] || std::abs(Xr[6] - Xr_km1[6]) > max_diff_Xr[6] || std::abs(Xr[7] - Xr_km1[7]) > max_diff_Xr[7] || std::abs(Xr[8] - Xr_km1[8]) > max_diff_Xr[8] || std::abs(Xr[9] - Xr_km1[9]) > max_diff_Xr[9] || std::abs(Xr[10] - Xr_km1[10]) > max_diff_Xr[10] || std::abs(Xr[11] - Xr_km1[11]) > max_diff_Xr[11] || std::abs(Xr[12] - Xr_km1[12]) > max_diff_Xr[12])
                {
                    auto msg_set = std_msgs::msg::Bool();
                    msg_set.data = true;
                    publisher_setparam->publish(msg_set);
                }

                /*auto msg = std_msgs::msg::Float32MultiArray();    
                publisher_->publish(msg);   
                // TODO definir el arreglo de datos a publicar con respecto a layout de la matriz y data
                
                //auto end = std::chrono::high_resolution_clock::now();
                //std::chrono::duration<double> elapsed = end - start;
                //double miliseconds = elapsed.count()*1000;
                //RCLCPP_INFO(this->get_logger(), "Exec time: %f", miliseconds);*/
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

    // void callback GPS local
    void callbackGpsLocalData(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if(armed == true){
        // Extracción de las posiciones y el ángulo (psi) a partir del mensaje
        float y = msg->pose.pose.position.x;
        float x = msg->pose.pose.position.y;
        float psi_rad = quat2EulerAngles_XYZ(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                             msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        psi_rad = -psi_rad + (M_PI / 2);
        if (psi_rad < 0){
            psi_rad += (2 * M_PI);
        }

        // Corrección de la posición debido al desplazamiento del GPS respecto al navío
        float dy = -0.2750;  // Desplazamiento en x de la antena
        float dx = 0.2625;   // Desplazamiento en y de la antena
        x = x + cos(psi_rad) * dx - sin(psi_rad) * dy;
        y = y + sin(psi_rad) * dx + cos(psi_rad) * dy;

        // --- Actualización de la orientación (psi) global y manejo de vueltas (laps) ---
        if (armed_act == false) {
            psi_ant = psi_rad;
            laps = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                psi = psi_rad;
            }
        } else {
            psi_act = psi_rad;
            if ((psi_act - psi_ant) > M_PI) {
                laps = laps - 1;
            } else if ((psi_act - psi_ant) < -M_PI) {
                laps = laps + 1;
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                psi = psi_act + 2 * M_PI * laps;
            }
            psi_ant = psi_act;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            bufferX.push_back(x);
            bufferY.push_back(y);
            bufferPsi.push_back(psi);
            bufferR.push_back(r);
            if (bufferX.size() > static_cast<std::vector<double>::size_type>(n_SG)) {
                bufferX.erase(bufferX.begin());
                bufferY.erase(bufferY.begin());
                bufferPsi.erase(bufferPsi.begin());
                bufferR.erase(bufferR.begin());
            }
        }   
    } else {
        psi_ant = 0;
        laps = 0;
    }
    armed_act = armed;
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
                // delta_diff_ant = delta_diff;
                // delta_mean_ant = delta_mean;
                float delta_diff = delta_left-delta_right, delta_mean = (delta_left+delta_right)/2.0;
                bufferBeta.push_back(beta_a);
                bufferdelta_mean.push_back(delta_mean);
                bufferdelta_diff.push_back(delta_diff);
                if (bufferdelta_mean.size() > static_cast<std::vector<double>::size_type>(n_SG)) {
                    bufferdelta_mean.erase(bufferdelta_mean.begin());
                    bufferdelta_diff.erase(bufferdelta_diff.begin());
                    bufferBeta.erase(bufferBeta.begin());
                }
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


    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            (void)param;            
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    void obtainVelocities() {
        // --- Cálculo de la derivada (velocidades en el sistema inercial) con Savitzky–Golay para 25 puntos ---
        float velX = 0.0f, velY = 0.0f;
        bool validSG = false;
        if (bufferX.size() == static_cast<std::vector<double>::size_type>(n_SG)) {

            constexpr size_t n_SG = 25;
            const float sgCoeffs25[n_SG] = {
                -0.013058f, -0.011245f, -0.009432f, -0.007619f, -0.005806f,
                -0.003993f, -0.002180f, -0.000367f,  0.001446f,  0.003259f,
                 0.005073f,  0.006886f,  0.000000f, -0.006886f, -0.005073f,
                -0.003259f, -0.001446f,  0.000367f,  0.002180f,  0.003993f,
                 0.005806f,  0.007619f,  0.009432f,  0.011245f,  0.013058f
            };
            
            // Se aplica la convolución utilizando los 25 coeficientes
            for (size_t i = 0; i < n_SG; i++) {
                velX += sgCoeffs25[i] * bufferX[i];
                velY += sgCoeffs25[i] * bufferY[i];
            }
            validSG = true;
        }
        // --- Fin del cálculo SG ---
       

        // --- Transformación a velocidades surge (u) y sway (v) ---
        if (validSG) {
            // Actualizamos los valores anteriores para la próxima iteración
            prev_u = u;
            prev_v = v;
            // Se utiliza el valor de psi correspondiente al centro de la ventana (índice 2)
            int valormedio = n_SG/2;
            float psi_central = bufferPsi[valormedio];
            // Matriz de rotación transpuesta:
            // [ u ]   [ cos(psi)   sin(psi) ] [ velX ]
            // [ v ] = [ -sin(psi)  cos(psi) ] [ velY ]
            float cos_psi = cos(psi_central);
            float sin_psi = sin(psi_central);

            u = cos_psi * velX + sin_psi * velY;
            v = -sin_psi * velX + cos_psi * velY;
            
            // RCLCPP_INFO(this->get_logger(), "Velocidades calculadas: velX = %f, velY = %f", velX, velY);
            // RCLCPP_INFO(this->get_logger(), "Surge (u): current = %f, previous = %f, diff = %f", u, prev_u, diff_u);
            // RCLCPP_INFO(this->get_logger(), "Sway (v): current = %f, previous = %f, diff = %f", v, prev_v, diff_v);
            
        }
        // --- Fin transformación y almacenamiento de surge y sway ---
    }

    // Función para calcular la ganancia de Kalman L(k)
    Eigen::MatrixXd calcularKalmanGain(const Eigen::MatrixXd& P_bar, 
        const Eigen::MatrixXd& A, 
        const Eigen::MatrixXd& R) {
    // Cálculo de la matriz innovación: S = A * P_bar * A^T + R
    Eigen::MatrixXd S = A * P_bar * A.transpose() + R;

    // Se asume que S es invertible. En aplicaciones reales, es recomendable
    // utilizar métodos numéricamente estables (por ejemplo, factorizaciones)
    Eigen::MatrixXd K = P_bar * A.transpose() * S.inverse();

    return K;
    }

    // Función para actualizar la covarianza usando la forma de Joseph
    Eigen::MatrixXd actualizarCovarianza(const Eigen::MatrixXd& P_bar, 
        const Eigen::MatrixXd& A, 
        const Eigen::MatrixXd& L, 
        const Eigen::MatrixXd& R) {
    // Dimensión del estado
    int n = P_bar.rows();
    // Matriz identidad de dimensión n x n
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(n, n);

    // Cálculo de la covarianza actualizada usando la forma de Joseph
    Eigen::MatrixXd P_actualizada = (I - L * A) * P_bar * (I - L * A).transpose() + L * R * L.transpose();

    return P_actualizada;
    }




    float psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;
    float r = 0.0;
    Eigen::Matrix<double, 1, 7> Xu;
    Eigen::Matrix<double, 1, 13> Xv, Xr;
    float t_s;
    int count=0;
    int count_set = 0;
    std::vector<double> bufferX;
    std::vector<double> bufferY;
    std::vector<double> bufferPsi;
    std::vector<double> bufferR;
    std::vector<double> bufferdelta_diff;
    std::vector<double> bufferdelta_mean;
    std::vector<double> bufferBeta;
    std::vector<double> bufferSigno;
    float prev_u = 0.0f;
    float prev_v = 0.0f;
    float u = 0.0f;
    float v = 0.0f;

    Eigen::Matrix<double, 1, 7> max_diff_Xu;
    Eigen::Matrix<double, 1, 13> max_diff_Xv;
    Eigen::Matrix<double, 1, 13> max_diff_Xr;

    // Inicializar ganancias de Kalman 
    /// matrizx de zeros 
    Eigen::Matrix<double, 7,1> L_u;
    Eigen::Matrix<double, 13,1> L_v;
    Eigen::Matrix<double, 13,1> L_r;
    // Matriz de covarianza de la predicción (P_bar)
    Eigen::Matrix<double, 7,7> P_bar_u;
    Eigen::Matrix<double, 13,13> P_bar_v;
    Eigen::Matrix<double, 13,13> P_bar_r;
    // Matriz de covarianza del ruido de medida (R)
    Eigen::Matrix<double, 1, 1> R;
    double R_value;
    
            
    int n_SG = 25;

    
    float Dz_up, Dz_down;  
 //------Params-------//
    std::string my_id;
    float Ts;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_gps_local;
    rclcpp::Subscription<mavros_msgs::msg::RCOut>::SharedPtr subscriber_rcout;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_param;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_kalmangains;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_velocities;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_setparam;

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
    auto node = std::make_shared<ObserverParamNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}