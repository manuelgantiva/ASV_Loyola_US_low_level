#include "rclcpp/rclcpp.hpp"
#include "asv_library/Zonotopo.h"
#include "mavros_msgs/msg/state.hpp"               //Interface state ardupilot
#include "geometry_msgs/msg/pose_stamped.hpp"       //Interface gps local data
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "std_msgs/msg/float32_multi_array.hpp"

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
        this-> declare_parameter("my_id", "ASV0");
          //---------Parámetros del ASV-------------------//
        this-> declare_parameter("Ts", 100.0);
        
        this-> declare_parameter("Xu", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xv", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xr", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        this-> declare_parameter("Max_n_psi", 0.01);
        this-> declare_parameter("Max_n_r", 0.01);
        this-> declare_parameter("Max_n_p", 0.02);
        this-> declare_parameter("Max_w_r", 0.01);
        this-> declare_parameter("Max_w_p", 0.02);

        this-> declare_parameter("q", 300);
        this-> declare_parameter("met", 1);

        this-> declare_parameter("Wr_di", std::vector<float>{1.0, 1.0, 1.0});
        this-> declare_parameter("Wp_di", std::vector<float>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        this-> declare_parameter("IMU_on", false);
        this-> declare_parameter("Sig_on", false);

        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        t_s = Ts/1000; // En segundos

        Xu = this->get_parameter("Xu").as_double_array();
        Xv = this->get_parameter("Xv").as_double_array();
        Xr = this->get_parameter("Xr").as_double_array();

        Max_n_psi = this->get_parameter("Max_n_psi").as_double();
        Max_n_r = this->get_parameter("Max_n_r").as_double(); // pequeños Metodo 2
        Max_n_p = this->get_parameter("Max_n_p").as_double();
        Max_w_r = this->get_parameter("Max_w_r").as_double();
        Max_w_p = this->get_parameter("Max_w_p").as_double();

        q = this->get_parameter("q").as_int();
        met = this->get_parameter("met").as_int();

        IMU_on = this->get_parameter("IMU_on").as_bool();
        Sig_on = this->get_parameter("Sig_on").as_bool();

        std::vector<double> Wr_di = this->get_parameter("Wr_di").as_double_array();
        std::vector<double> Wp_di = this->get_parameter("Wp_di").as_double_array();

        Ar << 1.0, t_s, 0.0,
                0.0, 1.0, t_s,
                0.0, 0.0, 1.0; 

        Cr << 1.0, 0.0, 0.0,
              0.0, 1.0, 0.0; 

        Bwr << 0.0,
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

        IGr << 0.0,
                0.0,
                0.0;

        IGp << 0.0,
              0.0,
              0.0,
              0.0,
              0.0,
              0.0;

        // Condiciones iniciales para los conjuntos
        Zr_prior= Zonotopo(VectorXd::Zero(3), MatrixXd::Identity(3,3));
        Zp_prior = Zonotopo(VectorXd::Zero(6), MatrixXd::Identity(6,6));

        Rr << Max_n_psi, 0.0,
              0.0, Max_n_r;
        Rp << Max_n_p, 0.0,
              0.0, Max_n_p;
        
        Qr << Max_w_r;
        Qp << Max_w_p, 0.0,
              0.0, Max_w_p;
 
        Wr << Wr_di[0], 0, 0,
                0, Wr_di[1], 0,
                0, 0, Wr_di[2];

        Wp << Wp_di[0], 0, 0, 0, 0, 0,
              0, Wp_di[1], 0, 0, 0, 0,
              0, 0, Wp_di[2], 0, 0, 0,
              0, 0, 0, Wp_di[3], 0, 0,
              0, 0, 0, 0, Wp_di[4], 0,
              0, 0, 0, 0, 0, Wp_di[5];

        R2T.setZero();
        Yp.setZero();
    
        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;


        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ObserverZonoNode::param_callback, this, _1));

        // suscribir a IMU 

        subscription_data = this->create_subscription<std_msgs::msg::Float32MultiArray>("/" + my_id + "/observer/data_sensors", rclcpp::SensorDataQoS(),
            std::bind(&ObserverZonoNode::calculateState, this, std::placeholders::_1),options_sensors_);

        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&ObserverZonoNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_state = this-> create_publisher<asv_interfaces::msg::StateObserver>("/" + my_id + "/observer/state_observer_zono",
                rclcpp::SensorDataQoS());
        publisher_state_min = this-> create_publisher<asv_interfaces::msg::StateObserver>("/" + my_id + "/observer/state_observer_zono_min",
                rclcpp::SensorDataQoS());
        publisher_state_max = this-> create_publisher<asv_interfaces::msg::StateObserver>("/" + my_id + "/observer/state_observer_zono_max",
                rclcpp::SensorDataQoS());
        publisher_obs = this-> create_publisher<geometry_msgs::msg::PoseStamped>("/" + my_id + "/observer/pose_zono",
                rclcpp::SensorDataQoS());
        publisher_sigmas = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/observer/sigmas_zono",1);
        
                                        
        RCLCPP_INFO(this->get_logger(), "Observer Zonotopos Node in %s has been started.", my_id.c_str());
    }

private:
    void calculateState(const std_msgs::msg::Float32MultiArray::SharedPtr msg_data)
    {
        auto msg = asv_interfaces::msg::StateObserver();
        if(armed==false){ 
            count=0;
            R2T.setZero();
            Zr_prior= Zonotopo(VectorXd::Zero(3), MatrixXd::Identity(3,3));
            Zp_prior = Zonotopo(VectorXd::Zero(6), MatrixXd::Identity(6,6));
            Sigmas.setZero();
        }else{
            if(count > 6){
                //ssauto start = std::chrono::high_resolution_clock::now();
                Vector <double, 2> Yp_i;
                Vector <double, 2> Yr_i;
                float delta_diff_i;
                float delta_mean_i;
                int beta_i;

                Yp_i << static_cast<double>(msg_data->data[0]),
                        static_cast<double>(msg_data->data[1]);
                Yr_i << static_cast<double>(msg_data->data[2]),
                        static_cast<double>(msg_data->data[3]);

                delta_diff_i = msg_data->data[4];
                delta_mean_i = msg_data->data[5];
                beta_i=static_cast<int>(msg_data->data[6]);


                float delta_mean_i_2 = delta_mean_i*delta_mean_i;
                float delta_diff_i_2 = delta_diff_i*delta_diff_i;
                float sum_1 = (delta_mean_i_2+(delta_diff_i_2/4.0));
                float sig;
                if(delta_diff_i>=0){
                    sig=1;
                }else{
                    sig=-1;
                }
                                
                // inicializar variables
                if(count==7){
                    Eigen::VectorXd cr0(3);
                    cr0 << Yr_i(0), 0.0, 0.0;  
                    Eigen::VectorXd cp0(6);
                    cp0 << Yp_i(0), Yp_i(1), 0.0, 0.0, 0.0, 0.0;  
                    Zr_prior = Zonotopo(cr0, Eigen::MatrixXd::Identity(3, 3));  // Amplia Zonotopo si no estoy seguro
                    Zp_prior = Zonotopo(cp0, Eigen::MatrixXd::Identity(6, 6));  // Amplia Zonotopo si no estoy seguro
                    Zr_next = Zonotopo(cr0, Eigen::MatrixXd::Identity(3, 3));  // Iniciar para sigmas
                    Zp_next = Zonotopo(cp0, Eigen::MatrixXd::Identity(6, 6));  // Iniciar para sigmas
                    count=count+1;
                }

                IGp(2,0) = (Xu[4]*sum_1)+(Xu[5]*delta_mean_i);
                IGp(3,0) = (Xv[8]*sum_1*(1-beta_i)*sig)+(Xv[9]*delta_mean_i*delta_diff_i)+(Xv[10]*delta_mean_i*(1-beta_i)*sig)+(Xv[11]*delta_diff_i/2.0);
                IGr(1,0) = (Xr[8]*sum_1*(1-beta_i)*sig)+(Xr[9]*delta_mean_i*delta_diff_i)+(Xr[10]*delta_mean_i*(1-beta_i)*sig)+(Xr[11]*delta_diff_i/2.0);

                if(Sig_on){
                    Sigmas(0) = (Xu[0] * Zp_next.c(2) * std::abs(Zp_next.c(2)) + Xu[1] * Zp_next.c(3) * Zr_next.c(1) + Xu[2] * Zr_next.c(1) * Zr_next.c(1)
                                + Xu[3] * Zp_next.c(2));
                    Sigmas(1) = (Xv[0] * Zp_next.c(3) * std::abs(Zp_next.c(3)) + Xv[1] * Zp_next.c(3) * std::abs(Zr_next.c(1)) + Xv[2] * Zr_next.c(1) * std::abs(Zp_next.c(3)) 
                                + Xv[3] * Zr_next.c(1) * std::abs(Zr_next.c(1)) + Xv[4] * Zp_next.c(2) * Zp_next.c(3) +Xv[5] * Zp_next.c(2) * Zr_next.c(1) 
                                + Xv[6] * Zp_next.c(3) +Xv[7] * Zr_next.c(1));
                    Sigmas(2) = (Xr[0] * Zp_next.c(3) * std::abs(Zp_next.c(3)) + Xr[1] * Zp_next.c(3) * std::abs(Zr_next.c(1)) + Xr[2] * Zr_next.c(1) * std::abs(Zp_next.c(3)) 
                                + Xr[3] * Zr_next.c(1) * std::abs(Zr_next.c(1)) + Xr[4] * Zp_next.c(2) * Zp_next.c(3) + Xr[5] * Zp_next.c(2) * Zr_next.c(1) 
                                + Xr[6] * Zp_next.c(3) + Xr[7] * Zr_next.c(1));
                }

                IGp(2,0) = (IGp(2,0) + Sigmas(0))*0.1;
                IGp(3,0) = (IGp(3,0) + Sigmas(1))*0.1;
                IGr(1,0) = (IGr(1,0) + Sigmas(2))*0.1;

                // Llamar al método de filtrado
                if(IMU_on){
                    Zr_next = Zonotopo::filteringR(Zr_prior, Yr_i, Cr, Rr, Eigen::MatrixXd::Identity(3, 3));
                }else{
                    Zr_next = Zonotopo::filteringPsi(Zr_prior, Yr_i.segment(0,1), Cr.block<1,3>(0,0), Rr.block<1,1>(0,0), Eigen::MatrixXd::Identity(3, 3));
                }
                
                //Calcular bandas rotacional
                MatrixXd br = rs_z(Zr_next);
                int nr = br.rows();
                std::vector<double> min_r(nr), max_r(nr);
                for (int oo = 0; oo < nr; ++oo) {
                    min_r[oo] = br(oo, 0);  // Primera columna de br (mínimos)
                    max_r[oo] = br(oo, 1);  // Segunda columna de br (máximos)
                }
    
                Zr_next.reduccionOrden(q,Wr);

                Zp_next = Zonotopo::filteringP(Zp_prior, Yp_i, Cp, Rp, Eigen::MatrixXd::Identity(6, 6));
                //Calcular bandas posicional
                MatrixXd bp = rs_z(Zp_next);
                int np = bp.rows();             
                std::vector<double> min_p(np), max_p(np);
                for (int oo = 0; oo < np; ++oo) {
                    min_p[oo] = bp(oo, 0);  // Primera columna de bp (mínimos)
                    max_p[oo] = bp(oo, 1);  // Segunda columna de bp (máximos)
                }

                Zp_next.reduccionOrden(2*q,Wp);

                VectorXd qr = Ar * Zr_next.c + IGr;
                MatrixXd Hr1 = Ar * Zr_next.H;
                MatrixXd Hr2 = Bwr * Qr;
                MatrixXd Hr(Hr1.rows(), Hr1.cols() + Hr2.cols());
                Hr << Hr1, Hr2;
                Zr_prior = Zonotopo(qr, Hr);

                if(met == 1){
                    Zp_prior = Zonotopo::prediction_Y(Ap,Zp_next,Yr_i(0)-Max_n_psi,Yr_i(0)+Max_n_psi,Bwp,Qp,IGp);
                }else{
                    Zp_prior = Zonotopo::prediction2_Y(Ap, t_s, Zp_next,Yr_i(0)-Max_n_r,Yr_i(0)+Max_n_r,Bwp,Qp,IGp,1);
                }

                msg.header.stamp = this->now();
                msg.header.frame_id = my_id; 

                msg.point.x=Zp_next.c(0);
                msg.point.y=Zp_next.c(1);
                msg.point.z=Zr_next.c(0);
                msg.velocity.x=Zp_next.c(2);
                msg.velocity.y=Zp_next.c(3);
                msg.velocity.z=Zr_next.c(1);
                msg.disturbances.x=Zp_next.c(4) + Sigmas(0);
                msg.disturbances.y=Zp_next.c(5) + Sigmas(1);
                msg.disturbances.z=Zr_next.c(2) + Sigmas(2);
                publisher_state->publish(msg);

                msg.point.x=min_p[0];
                msg.point.y=min_p[1];
                msg.point.z=min_r[0];
                msg.velocity.x=min_p[2];
                msg.velocity.y=min_p[3];
                msg.velocity.z=min_r[1];
                msg.disturbances.x=min_p[4] + Sigmas(0);
                msg.disturbances.y=min_p[5] + Sigmas(1);
                msg.disturbances.z=min_r[2] + Sigmas(2);
                publisher_state_min->publish(msg);

                msg.point.x=max_p[0];
                msg.point.y=max_p[1];
                msg.point.z=max_r[0];
                msg.velocity.x=max_p[2];
                msg.velocity.y=max_p[3];
                msg.velocity.z=max_r[1];
                msg.disturbances.x=max_p[4] + Sigmas(0);
                msg.disturbances.y=max_p[5] + Sigmas(1);
                msg.disturbances.z=max_r[2] + Sigmas(2);
                publisher_state_max->publish(msg);

                auto msg_obs = geometry_msgs::msg::PoseStamped();

                msg_obs.header.stamp = this->now();
                msg_obs.header.frame_id = "map_ned";
                msg_obs.pose.position.x= Zp_next.c(0);
                msg_obs.pose.position.y= Zp_next.c(1);
                msg_obs.pose.position.z= 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, Zr_next.c(0));
                msg_obs.pose.orientation.x = q.x();
                msg_obs.pose.orientation.y = q.y();
                msg_obs.pose.orientation.z = q.z();
                msg_obs.pose.orientation.w = q.w();
                publisher_obs->publish(msg_obs); 

                if(Sig_on){
                    auto msg_s = geometry_msgs::msg::Vector3();
                    msg_s.x = Zp_next.c(4); 
                    msg_s.y = Zp_next.c(5); 
                    msg_s.z = Zr_next.c(2); 
                    publisher_sigmas->publish(msg_s);
                }

                //auto end = std::chrono::high_resolution_clock::now();
                //std::chrono::duration<double> elapsed = end - start;
                //double miliseconds = elapsed.count()*1000;
                //RCLCPP_INFO(this->get_logger(), "Exec time: %f", miliseconds);
            }else{
                count=count+1;
            }
        }
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
            if (param.get_name() == "Max_n_psi"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value"); 
                    Max_n_psi = param.as_double();
                    Rr(0,0) = Max_n_psi;      
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
                    Rr(1,1) = Max_n_r;      
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
                    Qr << Max_w_r;
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
            if (param.get_name() == "met"){
                if(param.as_int() == 1 or param.as_int() == 2){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    met = this->get_parameter("met").as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be 1 or 2");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Wr_di"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Wr_di = param.as_double_array();
                    Wr << Wr_di[0], 0, 0,
                            0, Wr_di[1], 0,
                            0, 0, Wr_di[2];
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
            if (param.get_name() == "Xu"){
                if(param.as_double_array().size() == 6){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Xu = this->get_parameter("Xu").as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 6");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Xv"){
                if(param.as_double_array().size() == 12){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Xv = this->get_parameter("Xv").as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 12");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Xr"){
                if(param.as_double_array().size() == 12){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Xr = this->get_parameter("Xr").as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 12");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "IMU_on"){
                RCLCPP_INFO(this->get_logger(), "changed param value");
                IMU_on = this->get_parameter("IMU_on").as_bool();
            }
            if (param.get_name() == "Sig_on"){
                RCLCPP_INFO(this->get_logger(), "changed param value");
                Sig_on = this->get_parameter("Sig_on").as_bool();
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    float psi_act = 0.0, psi_ant = 0.0, psi_0 = 0.0, psi = 0.0;
    int status_gps, laps=0;
    bool armed = false, armed_act = false;
    float r = 0.0;

 //------Params-------//
    std::string my_id;
    float Ts, t_s; 

    std::vector<double> Xu, Xv, Xr;

    bool IMU_on, Sig_on;

    float Max_w_r, Max_w_p, Max_n_p, Max_n_r, Max_n_psi;

    float delta_diff;
    float delta_mean;
    int beta, count=0, q, met;  

    Matrix <double, 3,1> IGr; 
    Matrix <double, 6,1> IGp;
    Matrix <double, 2,2> R2T;
    Vector <double, 2> Yp;
    Vector <double, 3> Sigmas;  
    Matrix <double, 3,3> Ar;
    Matrix <double, 2,3> Cr;
    Matrix <double, 3,1> Bwr;
    Matrix <double, 6,6> Ap;
    Matrix <double, 2,6> Cp;
    Matrix <double, 6,2> Bwp;
    Matrix <double, 2,2> Rr;
    Matrix <double, 2,2> Rp;
    Matrix <double, 1,1> Qr;
    Matrix <double, 2,2> Qp;
    Matrix <double, 3,3> Wr;
    Matrix <double, 6,6> Wp;
    
    Zonotopo Zp_prior, Zr_prior, Zp_next, Zr_next;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_data;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_obs;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_min;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_max;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_sigmas;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;

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