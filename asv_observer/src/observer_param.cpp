#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"                  //Interface Imu data
#include "sensor_msgs/msg/nav_sat_fix.hpp"          //Interface gps global data
#include "std_msgs/msg/float64.hpp"                 //Interface yaw data topic compass_hdg
#include "mavros_msgs/msg/rc_out.hpp"               //Interface rc out pwm value actual
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/twist_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "nav_msgs/msg/odometry.hpp"                //Interface gps global local data
#include "std_msgs/msg/float32_multi_array.hpp"          // Interface coeficientes polinomio
#include "std_msgs/msg/bool.hpp"                    //Interface armed data
#include "rcl_interfaces/srv/set_parameters.hpp"     //Interface set parameters
#include "rcl_interfaces/msg/parameter.hpp"          //Interface parameter



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

        this-> declare_parameter("R", 0.01);

        // variacion de parámetros
        this-> declare_parameter("max_diff_Xu", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        this-> declare_parameter("max_diff_Xv", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});
        this-> declare_parameter("max_diff_Xr", std::vector<double>{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1});

        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        t_s = Ts/1000; // En segundos

        std::vector<double> Xu_vector = this->get_parameter("Xu").as_double_array();
        std::vector<double> Xv_vector = this->get_parameter("Xv").as_double_array();
        std::vector<double> Xr_vector = this->get_parameter("Xr").as_double_array();

        // info Xu_vector
        // RCLCPP_INFO(this->get_logger(),  "Valores de X linea 54 : %f, %f, %f.", Xu_vector[0], Xv_vector[0], Xr_vector[0]);

        Xu = Eigen::Map<Eigen::Matrix<double, 7, 1>>(Xu_vector.data());
        Xv = Eigen::Map<Eigen::Matrix<double, 13, 1>>(Xv_vector.data());
        Xr = Eigen::Map<Eigen::Matrix<double, 13, 1>>(Xr_vector.data());
        
        // info Xu
        // RCLCPP_INFO(this->get_logger(),  "Valores de X linea 61 : %f, %f, %f.", Xu(0,0), Xv(0,0), Xr(0,0));

        std::vector<double> diff_Xu = this->get_parameter("max_diff_Xu").as_double_array();
        std::vector<double> diff_Xv = this->get_parameter("max_diff_Xv").as_double_array();
        std::vector<double> diff_Xr = this->get_parameter("max_diff_Xr").as_double_array();

        max_diff_Xu = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diff_Xu.data());
        max_diff_Xv = Eigen::Map<Eigen::Matrix<double, 13, 1>>(diff_Xv.data());
        max_diff_Xr = Eigen::Map<Eigen::Matrix<double, 13, 1>>(diff_Xr.data());

        R_value = this->get_parameter("R").as_double();
        R.setZero();
        
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObserverParamNode::param_callback, this, _1));

        subscription_data = this->create_subscription<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/data_sensors", rclcpp::SensorDataQoS(),
            std::bind(&ObserverParamNode::calculateState, this, std::placeholders::_1),options_sensors_);

        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1, 
                std::bind(&ObserverParamNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        publisher_param = this-> create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/observer_param",
                    rclcpp::SensorDataQoS());
        publisher_kalmangains = this-> create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/kalman_gains",
                    rclcpp::SensorDataQoS());
        publisher_velocities = this-> create_publisher<geometry_msgs::msg::TwistStamped>("/" + my_id + "/observer/velocities",
                    rclcpp::SensorDataQoS());
        publisher_setparam = this-> create_publisher<std_msgs::msg::Bool>("/" + my_id + "/observer/set_param",
                    1);
        
        this-> client_setparam = this->create_client<rcl_interfaces::srv::SetParameters>("/" + my_id + "/observer/observer_zono_2/set_parameters");
           
        // std::vector<bool> myVector = {true, false, false};
        // threads_.push_back(std::thread(std::bind(&ObserverParamNode::callSetParametersService, this, Xu, Xv, Xr, myVector)));
                    
        RCLCPP_INFO(this->get_logger(), "Observer Parameters Node in %s has been started.", my_id.c_str());
    }

private:
    void calculateState(const std_msgs::msg::Float32MultiArray::SharedPtr msg_data)
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
                // Inicializar ganancias  1, 1, 1, 1, 1, 1, 1
                L_u = Eigen::Matrix<double, 7, 1> {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
                L_v = Eigen::Matrix<double, 13, 1> {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                L_r = Eigen::Matrix<double, 13, 1> {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                
                // Matriz de covarianza de la predicción (P_bar)
                P_bar_u = Eigen::MatrixXd::Identity(7, 7);
                P_bar_v = Eigen::MatrixXd::Identity(13, 13);
                P_bar_r = Eigen::MatrixXd::Identity(13, 13);
                // Matriz de covarianza del ruido de medida (R)
                R = Eigen::Matrix<double, 1, 1> {R_value};
                // RCLCPP_INFO(this->get_logger(),  "Valores de R : %f.", R(0,0));
            }
        }else{
            if(count > 6){
                
                
                int beta;
                double x, y, psi, r, delta_diff, delta_mean;
                x = static_cast<double>(msg_data->data[0]),
                y = static_cast<double>(msg_data->data[1]);
                psi = static_cast<double>(msg_data->data[2]),
                r = static_cast<double>(msg_data->data[3]);

                delta_diff = static_cast<double>(msg_data->data[4]);
                delta_mean = static_cast<double>(msg_data->data[5]);
                beta = static_cast<int>(msg_data->data[6]);


                updateBuffers(x, y, psi, r, delta_diff, delta_mean, beta);

                count = count + 1;

                if (count > 6 + n_SG){
                    double delta_mean_km1, delta_diff_km1, u_km1, v_km1, r_km1, u_k, v_k, r_k, beta_km1, d;
                    Eigen::Matrix<double,1 ,1> bu, bv, br;
                    bu.setZero();
                    bv.setZero();
                    br.setZero();

                    obtainVelocities();
                    // RCLCPP_INFO(this->get_logger(),  "Velocidades calculadas: %f, %f, %f", u, v, r);

                    int km1 = n_SG/2-1;
                    delta_diff_km1 = bufferdelta_diff[km1];
                    delta_mean_km1 = bufferdelta_mean[km1];
                    beta_km1 = bufferBeta[km1];
                    u_km1 = prev_u;
                    v_km1 = prev_v;
                    r_km1 = bufferR[km1];
                    u_k = u;
                    v_k = v;
                    r_k = bufferR[km1+1];

                    int sig;
                    if(delta_diff_km1==0){
                        sig=1;
                    }else{
                        sig=-1;
                    }

                    d = delta_mean_km1*delta_mean_km1 + 0.25*delta_diff_km1*delta_diff_km1;
                    // RCLCPP_INFO(this->get_logger(), "Antes de definir matrices A");
                    Eigen::Matrix<double, 1, 7> Au = {u_km1*abs(u_km1), v_km1*r_km1, r_km1*r_km1, u_km1, d, delta_mean_km1, 1};
                    
                    Eigen::Matrix<double, 1, 13> Av = {v_km1*abs(v_km1), v_km1*abs(r_km1), r_km1*abs(v_km1), r_km1*abs(r_km1), u_km1*v_km1, u_km1*r_km1, v_km1, r_km1, sig*(1-beta_km1)*d, delta_diff_km1*delta_mean_km1, sig*(1-beta_km1)*delta_mean_km1, delta_diff_km1/2, 1};
                    Eigen::Matrix<double, 1, 13> Ar = Av;

                    
                    bu << u_k - u_km1;
                
                    bv << v_k - v_km1;
                    br << r_k - r_km1;
                    

                    // Filtro de Kalman para obtener Xu, Xv, Xr
                    // RCLCPP_INFO(this->get_logger(), "Despues de definir matrices A, antes de X");
                    Eigen::Matrix<double, 7, 1> Xu_km1 = {Xu[0], Xu[1], Xu[2], Xu[3], Xu[4], Xu[5], Xu[6]};
                
                    Eigen::Matrix<double, 13, 1> Xv_km1 = {Xv[0], Xv[1], Xv[2], Xv[3], Xv[4], Xv[5], Xv[6], Xv[7], Xv[8], Xv[9], Xv[10], Xv[11], Xv[12]};
                    Eigen::Matrix<double, 13, 1> Xr_km1 = {Xr[0], Xr[1], Xr[2], Xr[3], Xr[4], Xr[5], Xr[6], Xr[7], Xr[8], Xr[9], Xr[10], Xr[11], Xr[12]};
                    

                    if(delta_mean_km1>0.5*delta_diff_km1 && delta_mean_km1>-0.5*delta_diff_km1){
                        // Etapa de Filtrado
                        Xu = Xu_km1 + L_u * (bu - Au*Xu_km1);
                        // Actualizar ganancias de Kalman
                        L_u = P_bar_u * Au.transpose() * (Au * P_bar_u * Au.transpose() + R).inverse();
                        // Actualizar matriz de covarianza (P_bar)
                        P_bar_u = (Eigen::MatrixXd::Identity(7, 7) - L_u * Au) * P_bar_u;
                    }
                    if(delta_diff_km1 != 0){
                        // Etapa de Filtrado
                        Xv = Xv_km1 + L_v * (bv - Av*Xv_km1);
                        Xr = Xr_km1 + L_r * (br - Ar*Xr_km1);
                        // Actualizar ganancias de Kalman
                        L_v = P_bar_v * Av.transpose() * (Av * P_bar_v * Av.transpose() + R).inverse();
                        L_r = P_bar_r * Ar.transpose() * (Ar * P_bar_r * Ar.transpose() + R).inverse();
                        // Actualizar matriz de covarianza (P_bar)
                        P_bar_v = (Eigen::MatrixXd::Identity(13, 13) - L_v * Av) * P_bar_v;
                        P_bar_r = (Eigen::MatrixXd::Identity(13, 13) - L_r * Ar) * P_bar_r;
                    }
                    
                    


                    
                    // // RCLCPP_INFO(this->get_logger(), "Despues de definir matrices X");
                    // if(delta_mean_km1>0.5*delta_diff_km1 && delta_mean_km1>-0.5*delta_diff_km1){
                    //     // Etapa de Filtrado
                    //     Xu = Xu_km1 + L_u * (bu - Au*Xu_km1);
                    //     // Actualizar ganancias de Kalman
                    //     L_u = P_bar_u * Au.transpose() * (Au * P_bar_u * Au.transpose() + R).inverse();
                    //     // Actualizar matriz de covarianza (P_bar)
                    //     P_bar_u = (Eigen::MatrixXd::Identity(7, 7) - L_u * Au) * P_bar_u;
                    // }else{
                    //     // Etapa de Filtrado
                    //     Xv = Xv_km1 + L_v * (bv - Av*Xv_km1);
                    //     Xr = Xr_km1 + L_r * (br - Ar*Xr_km1);
                    //     // Actualizar ganancias de Kalman
                    //     L_v = P_bar_v * Av.transpose() * (Av * P_bar_v * Av.transpose() + R).inverse();
                    //     L_r = P_bar_r * Ar.transpose() * (Ar * P_bar_r * Ar.transpose() + R).inverse();
                    //     // Actualizar matriz de covarianza (P_bar)
                    //     P_bar_v = (Eigen::MatrixXd::Identity(13, 13) - L_v * Av) * P_bar_v;
                    //     P_bar_r = (Eigen::MatrixXd::Identity(13, 13) - L_r * Ar) * P_bar_r;
                    // }

                
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


                    // Publicar los valores de las velocidades surge, sway y yaw
                    geometry_msgs::msg::TwistStamped msg_velocities;
                    msg_velocities.header.stamp = this->now();
                    msg_velocities.header.frame_id = "odom";
                    msg_velocities.twist.linear.x = u;
                    msg_velocities.twist.linear.y = v;
                    msg_velocities.twist.angular.z = r;
                    publisher_velocities->publish(msg_velocities);


                    // Publicar mensaje para setear los parámetros en el observador de estados
                    count_set = count_set + 1;
                    // Eigen::Matrix<double, 7, 1> max_diff_Xu = {max_diff_Xu[0], max_diff_Xu[1], max_diff_Xu[2], max_diff_Xu[3], max_diff_Xu[4], max_diff_Xu[5], max_diff_Xu[6]};
                    // Eigen::Matrix<double, 13, 1> max_diff_Xv = {max_diff_Xv[0], max_diff_Xv[1], max_diff_Xv[2], max_diff_Xv[3], max_diff_Xv[4], max_diff_Xv[5], max_diff_Xv[6], max_diff_Xv[7], max_diff_Xv[8], max_diff_Xv[9], max_diff_Xv[10], max_diff_Xv[11], max_diff_Xv[12]};
                    // Eigen::Matrix<double, 13, 1> max_diff_Xr = {max_diff_Xr[0], max_diff_Xr[1], max_diff_Xr[2], max_diff_Xr[3], max_diff_Xr[4], max_diff_Xr[5], max_diff_Xr[6], max_diff_Xr[7], max_diff_Xr[8], max_diff_Xr[9], max_diff_Xr[10], max_diff_Xr[11], max_diff_Xr[12]};
                    auto msg_set = std_msgs::msg::Bool();
                    msg_set.data = false;
                    bool set_Xu, set_Xv, set_Xr;
                    set_Xu = false;
                    set_Xv = false;
                    set_Xr = false;

                    // vectores de diferencias
                    Eigen::Matrix<double, 7, 1> diff_Xu = Xu - Xu_km1;
                    Eigen::Matrix<double, 13, 1> diff_Xv = Xv - Xv_km1;
                    Eigen::Matrix<double, 13, 1> diff_Xr = Xr - Xr_km1;

                    
                    // RCLCPP_INFO(this->get_logger(), "Diferencias en Xu: %f, %f, %f, %f, %f, %f, %f", diff_Xu(0,0), diff_Xu(1,0), diff_Xu(2,0), diff_Xu(3,0), diff_Xu(4,0), diff_Xu(5,0), diff_Xu(6,0));
                    // RCLCPP_INFO(this->get_logger(), "Diferencias en Xv: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", diff_Xv(0,0), diff_Xv(1,0), diff_Xv(2,0), diff_Xv(3,0), diff_Xv(4,0), diff_Xv(5,0), diff_Xv(6,0), diff_Xv(7,0), diff_Xv(8,0), diff_Xv(9,0), diff_Xv(10,0), diff_Xv(11,0), diff_Xv(12,0));
                    // RCLCPP_INFO(this->get_logger(), "Diferencias en Xr:  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f", diff_Xr(0,0), diff_Xr(1,0), diff_Xr(2,0), diff_Xr(3,0), diff_Xr(4,0), diff_Xr(5,0), diff_Xr(6,0), diff_Xr(7,0), diff_Xr(8,0), diff_Xr(9,0), diff_Xr(10,0), diff_Xr(11,0), diff_Xr(12,0));


                    // Verifica las diferencias en Xu
                    for (int i = 0; i < Xu.size(); ++i) {
                        diff_Xu(i,0) = Xu[i] - Xu_km1[i];
                        if (std::abs(Xu[i] - Xu_km1[i]) > max_diff_Xu[i]) {
                            set_Xu = true;
                            // break;
                        }
                    }

                    // Solo continúa si aún no se ha superado ningún umbral: verifica Xv
                    for (int i = 0; i < Xv.size(); ++i) {
                        diff_Xv(i,0) = Xv[i] - Xv_km1[i];
                        if (std::abs(Xv[i] - Xv_km1[i]) > max_diff_Xv[i]) {
                            set_Xv = true;
                            // break;
                        }
                    }

                    // Si aún no se ha superado el umbral, verifica Xr
                    for (int i = 0; i < Xr.size(); ++i) {
                        diff_Xr(i,0) = Xr[i] - Xr_km1[i];
                        if (std::abs(Xr[i] - Xr_km1[i]) > max_diff_Xr[i]) {
                            set_Xr = true;
                            // break;
                        }
                    }


                    // si se ha superado el número de iteraciones
                    // if (count_set > 600)
                    // { 
                    //     set_Xu = true;
                    //     set_Xv = true;
                    //     set_Xr = true;
                    //     count_set = 0;
                    // }

                    // Publicamos el mensaje
                    publisher_kalmangains->publish(msg_kg);
                    publisher_param->publish(msg);  
                    
                    if(set_Xu || set_Xv || set_Xr){
                        msg_set.data = true;
                        // Publicamos el mensaje
                        publisher_setparam->publish(msg_set);
                        std::vector<bool> myVector = {set_Xu, set_Xv, set_Xr};
                        threads_.push_back(std::thread(std::bind(&ObserverParamNode::callSetParametersService, this, Xu, Xv, Xr, myVector)));
                    }
                }
            }
            else if (count == 6){
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    bufferX.clear();
                    bufferY.clear();
                    bufferPsi.clear();
                    bufferR.clear();
                    bufferdelta_diff.clear();
                    bufferdelta_mean.clear();
                    // Inicializar ganancias  1, 1, 1, 1, 1, 1, 1
                    L_u = Eigen::Matrix<double, 7, 1> {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
                    L_v = Eigen::Matrix<double, 13, 1> {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                    L_r = Eigen::Matrix<double, 13, 1> {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
                    
                    // Matriz de covarianza de la predicción (P_bar)
                    P_bar_u = Eigen::MatrixXd::Identity(7, 7);
                    P_bar_v = Eigen::MatrixXd::Identity(13, 13);
                    P_bar_r = Eigen::MatrixXd::Identity(13, 13);
                    // Matriz de covarianza del ruido de medida (R)
                    R = Eigen::Matrix<double, 1, 1> {R_value};
                    RCLCPP_INFO(this->get_logger(),  "Valores de R : %f.", R(0,0));
                }
                count=count+1;
            }
            else{
                count=count+1;
            }
        }
    }

    // void fill the buffers
    void updateBuffers(double x, double y, double psi, double r, double delta_diff, double delta_mean, int beta)
    {
            bufferX.push_back(x);
            bufferY.push_back(y);
            bufferPsi.push_back(psi);
            bufferR.push_back(r);
            bufferdelta_diff.push_back(delta_diff);
            bufferdelta_mean.push_back(delta_mean);
            bufferBeta.push_back(beta);
            if (bufferX.size() > static_cast<std::vector<double>::size_type>(n_SG)) {
                bufferX.erase(bufferX.begin());
                bufferY.erase(bufferY.begin());
                bufferPsi.erase(bufferPsi.begin());
                bufferR.erase(bufferR.begin());
                bufferdelta_diff.erase(bufferdelta_diff.begin());
                bufferdelta_mean.erase(bufferdelta_mean.begin());
                bufferBeta.erase(bufferBeta.begin());
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
                -0.019389f, 0.014035f, 0.022376f, 0.015544f, 0.001269f,
                -0.014681f, -0.02828f, -0.03704f,  -0.03975f,  -0.03633f,
                -0.02754f,  -0.01481f,  0.000000f, 0.01481f, 0.02754f,
                0.03633f, 0.03975f,  0.03704f,  0.02828f,  0.014681f,
                -0.001269f,  -0.015544f,  -0.022376f,  -0.014035f,  0.019389f
            };
            
            // Se aplica la convolución utilizando los 25 coeficientes
            for (size_t i = 0; i < n_SG; i++) {
                velX += sgCoeffs25[i] * bufferX[i] / t_s;
                velY += sgCoeffs25[i] * bufferY[i] / t_s;
            }
            validSG = true;
        }
        // --- Fin del cálculo SG ---
       

        // --- Transformación a velocidades surge (u) y sway (v) ---
        if (validSG) {
            // Actualizamos los valores aresultnteriores para la próxima iteración
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

    void callSetParametersService(Eigen::Matrix<double, 7, 1>& Xu, Eigen::Matrix<double, 13, 1>& Xv,
                                Eigen::Matrix<double, 13, 1>& Xr, std::vector<bool> setters_X) {
        while (!client_setparam->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }

        auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();

        if (setters_X[0]) {
            std::vector<double> Xu_vector;
            Xu_vector.assign(Xu.data(), Xu.data() + 6);
            auto params = rcl_interfaces::msg::Parameter();
            params.name = "Xu";
            params.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            params.value.double_array_value = Xu_vector;           
            request -> parameters.push_back(params);
        }if (setters_X[1]) {
            std::vector<double> Xv_vector;
            Xv_vector.assign(Xv.data(), Xv.data() + 12);
            auto params = rcl_interfaces::msg::Parameter();
            params.name = "Xv";
            params.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            params.value.double_array_value = Xv_vector;           
            request -> parameters.push_back(params);
        }if (setters_X[2]) {
            std::vector<double> Xr_vector;
            Xr_vector.assign(Xr.data(), Xr.data() + 12);
            auto params = rcl_interfaces::msg::Parameter();
            params.name = "Xr";
            params.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY;
            params.value.double_array_value = Xr_vector;           
            request -> parameters.push_back(params);
        }        
        client_setparam->async_send_request(request, std::bind(&ObserverParamNode::setParamResponse, this, std::placeholders::_1)); 
    }

    void setParamResponse(rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFutureWithRequest future) {
        try {
            auto result = future.get();
            result.first->parameters.size();
            for (size_t i = 0; i < result.first->parameters.size(); ++i) {
                if (result.second->results[i].successful) {
                    // RCLCPP_INFO(this->get_logger(), "Parameter %s set successfully", result.first->parameters[i].name.c_str());
                } else {
                    // RCLCPP_INFO(this->get_logger(), "Parameter %s could not be set", result.first->parameters[i].name.c_str());
                }
            }
        } catch (const std::exception& e) {
            RCLCPP_INFO(this->get_logger(), "Service call failed");
        }
    }

    float psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;
    float r = 0.0;
    Eigen::Matrix<double, 7, 1> Xu;
    Eigen::Matrix<double, 13, 1> Xv, Xr;
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

    Eigen::Matrix<double, 7, 1> max_diff_Xu;
    Eigen::Matrix<double, 13, 1> max_diff_Xv;
    Eigen::Matrix<double, 13, 1> max_diff_Xr;

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

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_data;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_param;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_kalmangains;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_velocities;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_setparam;

    rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr client_setparam;

    rclcpp::TimerBase::SharedPtr timer_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
    std::vector<std::thread> threads_;
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