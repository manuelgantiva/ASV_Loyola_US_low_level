#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"  
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/state_neighbor.hpp"     //Interface state neighbor
#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <mutex>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <sstream>
#include <string>


using namespace std;
using namespace Eigen;

using std::placeholders::_1;

const int PWMMAX = 1900;
const int PWMMIN = 1100;

// Declaración de contantes

class LyapHlcNode : public rclcpp::Node
{
public:
    LyapHlcNode() : Node("lyap_hlc")
    { 
        std::string my_id; 

        //---------Parámetros del HLC-------------------// 

        memory_u.assign(4, 0.0);
        memory_r.assign(4, 0.0);

        // Declare parameters
        this->declare_parameter("my_id", "ASV0"); 
        
        this->declare_parameter("taud", 1.0);
        this->declare_parameter("Ts", 100.0);
        this->declare_parameter("lambda", 20.0);
        this->declare_parameter("zeta", 0.95);
        this->declare_parameter("gamma", 0.75);
        this->declare_parameter("sigma_h", 0.5);
        this->declare_parameter("mu_u", 2.0);
        this->declare_parameter("mu_r", 0.09);
        this->declare_parameter("k_d", 3.0);
        this->declare_parameter("k_theta", 3.0);
        this->declare_parameter("m11", 1.0);
        this->declare_parameter("m33_bar", 1.0);
        this->declare_parameter("m22", 1.0);
        this->declare_parameter("k_u", 0.49);
        this->declare_parameter("k_r", 0.9);
        this->declare_parameter("Sigma", 0.08);
        this->declare_parameter("d_cn", 5.5);
        this->declare_parameter("theta_cn", 3*M_PI/8);
        this->declare_parameter("d_cl", 4.5);
        this->declare_parameter("theta_cl", M_PI/8);
        this->declare_parameter("Kd", 0.08);
        this->declare_parameter("Ktheta", 0.1);
        this->declare_parameter("b_dinf", 0.05);
        this->declare_parameter("b_thetainf", 0.05);
        this->declare_parameter("ref_d", 5.0);
        this->declare_parameter("ref_theta", 0.29*M_PI);
        this->declare_parameter("eps_i", 0.1);

        this-> declare_parameter("mf0", 0.0013545);
        this-> declare_parameter("mf1", 6.0977);
        this-> declare_parameter("mf2", 0.0);
        this-> declare_parameter("mf3", -2.769);
        this-> declare_parameter("mf4", 0.0);
        this-> declare_parameter("mf5", -1.0978);
        this-> declare_parameter("mr0", -0.0059858);
        this-> declare_parameter("mr1", 6.1789);
        this-> declare_parameter("mr2", 0.20095);
        this-> declare_parameter("mr3", -5.1266);
        this-> declare_parameter("mr4", 1.048);
        this-> declare_parameter("mr5", -2.5286);
        this-> declare_parameter("df0", 0.0);
        this-> declare_parameter("df1", 0.0);
        this-> declare_parameter("df2", 6.3681);
        this-> declare_parameter("df3", 0.0);
        this-> declare_parameter("df4", 8.2298);
        this-> declare_parameter("df5", 0.0);
        this-> declare_parameter("dr0", 0.030548);
        this-> declare_parameter("dr1", -2.8142);
        this-> declare_parameter("dr2", 5.3685);
        this-> declare_parameter("dr3", 27.237);
        this-> declare_parameter("dr4", 4.2689);
        this-> declare_parameter("dr5", 13.881);

        this-> declare_parameter("IGumax_ff", 0.17794);
        this-> declare_parameter("IGumax_rf", 0.08897),
        this-> declare_parameter("IGumin_rf", -0.07331);
        this-> declare_parameter("IGrmax_ff", 0.14128);
        this-> declare_parameter("IGrmax_rf", 0.22898);

        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        // Get parameters
        my_id = this->get_parameter("my_id").as_string();
        
        taud = this->get_parameter("taud").as_double();
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        lambda = this->get_parameter("lambda").as_double();
        zeta = this->get_parameter("zeta").as_double();
        gamma = this->get_parameter("gamma").as_double();
        sigma_h = this->get_parameter("sigma_h").as_double();
        mu_u = this->get_parameter("mu_u").as_double();
        mu_r = this->get_parameter("mu_r").as_double();
        k_d = this->get_parameter("k_d").as_double();
        k_theta = this->get_parameter("k_theta").as_double();
        m11 = this->get_parameter("m11").as_double();
        m33_bar = this->get_parameter("m33_bar").as_double();
        m22 = this->get_parameter("m22").as_double();
        k_u = this->get_parameter("k_u").as_double();
        k_r = this->get_parameter("k_r").as_double();
        Sigma = this->get_parameter("Sigma").as_double();
        d_cn = this->get_parameter("d_cn").as_double();
        theta_cn = this->get_parameter("theta_cn").as_double();
        d_cl = this->get_parameter("d_cl").as_double();
        theta_cl = this->get_parameter("theta_cl").as_double();
        Kd = this->get_parameter("Kd").as_double();
        Ktheta = this->get_parameter("Ktheta").as_double();
        b_dinf = this->get_parameter("b_dinf").as_double();
        b_thetainf = this->get_parameter("b_thetainf").as_double();
        ref_d = this->get_parameter("ref_d").as_double();
        ref_theta = this->get_parameter("ref_theta").as_double();
        eps_i = this->get_parameter("eps_i").as_double();

        mf0 = this->get_parameter("mf0").as_double();
        mf1 = this->get_parameter("mf1").as_double();
        mf2 = this->get_parameter("mf2").as_double();
        mf3 = this->get_parameter("mf3").as_double();
        mf4 = this->get_parameter("mf4").as_double();
        mf5 = this->get_parameter("mf5").as_double();
        mr0 = this->get_parameter("mr0").as_double();
        mr1 = this->get_parameter("mr1").as_double();
        mr2 = this->get_parameter("mr2").as_double();
        mr3 = this->get_parameter("mr3").as_double();
        mr4 = this->get_parameter("mr4").as_double();
        mr5 = this->get_parameter("mr5").as_double();
        df0 = this->get_parameter("df0").as_double();
        df1 = this->get_parameter("df1").as_double();
        df2 = this->get_parameter("df2").as_double();
        df3 = this->get_parameter("df3").as_double();
        df4 = this->get_parameter("df4").as_double();
        df5 = this->get_parameter("df5").as_double();
        dr0 = this->get_parameter("dr0").as_double();
        dr1 = this->get_parameter("dr1").as_double();
        dr2 = this->get_parameter("dr2").as_double();
        dr3 = this->get_parameter("dr3").as_double();
        dr4 = this->get_parameter("dr4").as_double();
        dr5 = this->get_parameter("dr5").as_double();

        IGumax_ff = this->get_parameter("IGumax_ff").as_double();
        IGumax_rf = this->get_parameter("IGumax_rf").as_double();
        IGumin_rf = this->get_parameter("IGumin_rf").as_double();
        IGrmax_ff = this->get_parameter("IGrmax_ff").as_double();
        IGrmax_rf = this->get_parameter("IGrmax_rf").as_double();

        Dz_2  = this->get_parameter("Dz_up").as_double();
        Dz_1 = this->get_parameter("Dz_down").as_double();
        p = 1 - Dz_2;
        q = -1 - Dz_1;

        // Inicialización de flags y estados iniciales
        isFirstStep_HGO = true;
        isFirstStep_EHG = true;
        isFirstStep_DSC = true;
        h_hat = 0.0;
        alpha_fui = 0.0;
        alpha_fri = 0.0;
        
        // Inicialización de vectores de referencia para DSC
        R1 = Eigen::Vector2d(1.0, 0.0);
        R2 = Eigen::Vector2d(0.0, 1.0);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group = cb_group_sensors_;

        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(Ts * 1000.0)),
                std::bind(&LyapHlcNode::calculateHighLevelController, this), cb_group_obs_);

        // TODO: crear un param callback que actualice ciertos datos
        // params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&LyapHlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&LyapHlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        // subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
        //     "/" + my_id + "/control/reference_hlc", 1, std::bind(&LyapHlcNode::callbackVelReference,
        //     this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&LyapHlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        subscriber_state_neighbor_ = this-> create_subscription<asv_interfaces::msg::StateNeighbor>(
            "/" + my_id + "/neighbors/output_leader",rclcpp::SensorDataQoS(), std::bind(&LyapHlcNode::callbackNeighbor,
            this, std::placeholders::_1), options_sensors_);

        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/" + my_id + "/control/pwm_value_ifac",
                    10);
        publisher_IG = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/IG_lyap",1);
        
        }
        
private:
    void calculateHighLevelController()
    {
        
        if(armed==false){
            count = 0;
            // TODO: reiniciar datos que varian con el tiempo
        }else{
        auto msg = asv_interfaces::msg::PwmValues();
        auto msg_Igu = geometry_msgs::msg::Vector3();
        auto msg_Igr = geometry_msgs::msg::Vector3();
        auto msg_Ig = geometry_msgs::msg::Vector3();
        float zone;
        d = 0.0; 
        theta = 0.0;
            if(count > 7){
        float x_hat_i, y_hat_i, psi_hat_i, u_hat_i, v_hat_i, r_hat_i, sig_u_i, sig_v_i, sig_r_i;
        float x_hat_l_i, y_hat_l_i, psi_hat_l_i, u_hat_l_i, v_hat_l_i, r_hat_l_i;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat_i = x_hat;
            y_hat_i = y_hat;
            psi_hat_i = psi_hat;
            u_hat_i = u_hat;
            v_hat_i = v_hat;
            r_hat_i = r_hat;
            sig_u_i = sig_u;
            sig_v_i = sig_v;
            sig_r_i = sig_r;

            x_hat_l_i = x_hat_l;
            y_hat_l_i = y_hat_l;
            psi_hat_l_i = psi_hat_l;
            u_hat_l_i = u_hat_l;
            v_hat_l_i = v_hat_l;
            r_hat_l_i = r_hat_l;
            
        }
        
        Eigen::Vector<double, 6> Xf_est;
        Xf_est.setZero();
        Xf_est << x_hat_i, y_hat_i, psi_hat_i, u_hat_i, v_hat_i, r_hat_i;
        std::stringstream ss;
        ss << "Xf_est: " << Xf_est.transpose(); // transpose() per stamparlo in riga

        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        
        Eigen::Vector<double, 6> Xl_est;
        Xl_est.setZero();
        // (Xl_est << 1.5, 2.5, 0.9, 0.0, 3.5, 0.3;)
        Xl_est << x_hat_l_i, y_hat_l_i, psi_hat_l_i, u_hat_l_i, v_hat_l_i, r_hat_l_i;
        std::stringstream ss1;
        ss1 << "Xl_est: " << Xl_est.transpose(); // transpose() per stamparlo in riga

        RCLCPP_INFO(this->get_logger(), "%s", ss1.str().c_str());
        Eigen::Vector3d sigmaf_est;
        sigmaf_est.setZero();
        sigmaf_est << sig_u_i, sig_v_i, sig_r_i;
        double t = this->now().seconds() + this->now().nanoseconds(); 

        // --- Rotational matrices --- (From Simulator to paper)
        
        Eigen::Matrix3d Rs2pg, Rs2pb;
        Rs2pg.setZero();
        Rs2pb.setZero();
        Rs2pg << 0, 1, 0,
                 1, 0, 0,
                 0, 0, -1;
        Rs2pb << 1, 0, 0,
                 0, -1, 0,
                 0, 0, -1;
         
        Xf_est.segment(0, 3) = Rs2pg * Xf_est.segment(0, 3);
        Xf_est.segment(3, 3) = Rs2pb * Xf_est.segment(3, 3);
        // std::stringstream ss2;
        // ss2 << "Xf_est2: " << Xf_est.segment(0, 6).transpose(); // transpose() per stamparlo in riga

        // RCLCPP_INFO(this->get_logger(), "%s", ss2.str().c_str());

        Xl_est.segment(0, 3) = Rs2pg * Xl_est.segment(0, 3);
        Xl_est.segment(3, 3) = Rs2pb * Xl_est.segment(3, 3);
        sigmaf_est = Rs2pb * sigmaf_est;

        // --- Coordinate Transformation ---
        
        Eigen::Vector3d Xf_bar = CoordinateTransformation(Xf_est);
        Eigen::Vector3d Xl_bar = CoordinateTransformation(Xl_est);
        
        // std::stringstream ss2;
        // ss2 << "Xf_bar: " << Xf_bar.transpose(); // transpose() per stamparlo in riga

        // RCLCPP_INFO(this->get_logger(), "%s", ss2.str().c_str());
        // std::stringstream ss3;
        // ss3 << "Xl_bar: " << Xl_bar.transpose(); // transpose() per stamparlo in riga

        // RCLCPP_INFO(this->get_logger(), "%s", ss3.str().c_str());
        //RCLCPP_INFO(this->get_logger(), "Xf_bar: %d and Xl_bar:%d", Xf_bar, Xl_bar);

        double xf = Xf_bar(0);
        double yf = Xf_bar(1);
        
        double psi = Xf_est(2);
        double xl = Xl_bar(0);
        double yl = Xl_bar(1);
        //RCLCPP_INFO(this->get_logger(), "coseno di psi: %d", std::cos(psi));
        double e1 = std::cos(psi) * (xl - xf) + std::sin(psi) * (yl - yf);
        double e2 = -std::sin(psi) * (xl - xf) + std::cos(psi) * (yl - yf);
        //RCLCPP_INFO(this->get_logger(), "e1: %.5f and e2: %.5f", e1, e2);
        double d = std::sqrt(std::pow(xl - xf, 2) + std::pow(yl - yf, 2)); 
        double theta = std::atan2(e2, e1);

        RCLCPP_INFO(this->get_logger(), "distance: %.5f and theta: %.5f", d, theta);
        
        Eigen::Vector2d ref;
        ref.setZero();
        ref << d, theta;

        double e_d = d - ref_d;
        double e_theta = theta - ref_theta;

        RCLCPP_INFO(this->get_logger(), "e_d: %.5f and e_theta: %.5f", e_d, e_theta);


        Eigen::Vector4d beta = betaFunction(t);
        //RCLCPP_INFO(this->get_logger(), "beta_d(1): %.5f, beta_d(2): %.5f, beta_theta(1): %.5f", beta(0), beta(1), beta(2), beta(3));

        Eigen::Vector2d p_dot_l_est = HighGainObserver(Xl_bar);
        //RCLCPP_INFO(this->get_logger(), "p_dot_l_est %.5f", p_dot_l_est(0));
       
        Eigen::Vector2d q = Compute_q(beta, e_d, e_theta); 
        //RCLCPP_INFO(this->get_logger(), "q_d: %.5f and q_theta: %.5f", q(0), q(1));

        double H_hat = ComputeErrorHG(beta, d, e_d, e_theta);
        RCLCPP_INFO(this->get_logger(), "H_hat %.5f", H_hat);

        double var1_u = std::pow(beta(0), 2) / (2 * M_PI * e_d);
        double var2_u = (2 * beta(2) / beta(0)) + k_d;
        double var3_u = std::sin(M_PI * std::pow(e_d, 2)) / std::pow(beta(0), 2);
        double var4_u = (beta(2) * e_d) / beta(0);
        double var5_u = Xf_bar(2) * std::sin(theta);
        double var6_u = Xl_est(3); //p_dot_l_est.dot(R1);
        double var7_u = H_hat * std::tanh(e_d * q(0) * H_hat / Sigma);

        double var1_r = std::pow(beta(1), 2) / (2 * M_PI * e_theta);
        double var2_r = (2 * beta(3) / beta(1)) + k_theta;
        double var3_r = std::sin(M_PI * std::pow(e_theta, 2) / std::pow(beta(1), 2));
        double var4_r = (beta(3) * e_theta) / beta(1);
        
        double var5_r = Xf_est(3) * std::sin(theta);
        double var6_r = Xf_bar(2) * std::cos(theta);
        double var7_r = Xl_est(5); //p_dot_l_est.dot(R2);
        double var8_r = (H_hat / d) * std::tanh(e_theta * q(1) * H_hat / (Sigma * d));

        double alpha_ui = (var1_u * var2_u * var3_u - var4_u - var5_u + var6_u + var7_u);
        double alpha_ri = var1_r * var2_r * var3_r - var4_r + (var5_r - var6_r + var7_r) / d + var8_r;
        RCLCPP_INFO(this->get_logger(), "alpha_ui: %.5f and alpha_ri: %.5f", alpha_ui, alpha_ri);

    
        Eigen::Vector2d alpha_f = DSC(alpha_ui,alpha_ri, e_d, e_theta, q, theta);
        RCLCPP_INFO(this->get_logger(), "alpha_fu: %.5f and alpha_fr: %.5f", alpha_f(0), alpha_f(1));

        double e21 = Xf_est(3) - alpha_f(0); 
        double e22 = Xf_est(5) - alpha_f(1); 
        RCLCPP_INFO(this->get_logger(), "e21: %.5f and e22:%.5f", e21, e22);
        Eigen::Vector2d e2i;
        e2i.setZero();
        e2i << e21, e22;
        double e_alpha1 = alpha_f(0) - alpha_ui;
        double e_alpha2 = alpha_f(1) - alpha_ri;
        RCLCPP_INFO(this->get_logger(), "e_alpha1: %.5f and e_alpha2: %.5f", e_alpha1, e_alpha2);
        Eigen::Vector2d e_alpha;
        e_alpha.setZero();
        e_alpha << e_alpha1, e_alpha2;

        double IG_u = (-k_u * e21 - e_alpha1 / mu_u + 2 * e_d * q(0) * std::cos(theta)) - sigmaf_est(0);
        double IG_r = (-k_r * e22 - e_alpha2 / mu_r + 2 * e_theta * q(1)) - sigmaf_est(2);

        
        Eigen::Matrix<double, 3, 1> IG_real;
        IG_real.setZero();
        IG_real << IG_u, 0, IG_r;
        
        IG_real = Rs2pb.transpose() * IG_real;
        IG_u = IG_real(0);
        IG_r = IG_real(2);
        // Aquí se publicaría Tau_real usando el publisher (si estuviera habilitado)
        // New Code
        float m, d_;
        
        if(IG_u>IGumax_ff){
            IG_u=IGumax_ff;
        }

        if(IG_u<IGumin_rf){
            IG_u=IGumin_rf;
        }

        if(IG_r>IGrmax_rf){
            IG_r=IGrmax_rf;
        }

        if(IG_r<-IGrmax_rf){
            IG_r=-IGrmax_rf;
        }
        
        if(IG_u>IGumax_rf){
            // Zona Roja
            m = mf0+mf1*IG_u+mf2*IG_r+mf3*IG_u*IG_u+mf4*IG_u*IG_r+mf5*IG_r*IG_r;
            d_ = df0+df1*IG_u+df2*IG_r+df3*IG_u*IG_u+df4*IG_u*IG_r+df5*IG_r*IG_r;
            zone = 0;
        }else if(IG_r>IGrmax_ff){
            // Zona Azul
            m = mr0+mr1*IG_u+mr2*IG_r+mr3*IG_u*IG_u+mr4*IG_u*IG_r+mr5*IG_r*IG_r;
            d_ = dr0+dr1*IG_u+dr2*IG_r+dr3*IG_u*IG_u+dr4*IG_u*IG_r+dr5*IG_r*IG_r;
            zone = 1;
        }else if(IG_r<-IGrmax_ff){
            // Zona Verde
            m = mr0+mr1*IG_u-mr2*IG_r+mr3*IG_u*IG_u-mr4*IG_u*IG_r+mr5*IG_r*IG_r;
            d_ = dr0-dr1*IG_u+dr2*IG_r-dr3*IG_u*IG_u+dr4*IG_u*IG_r-dr5*IG_r*IG_r;
            zone = -1;
        }else{
            // Zona Roja
            m = mf0+mf1*IG_u+mf2*IG_r+mf3*IG_u*IG_u+mf4*IG_u*IG_r+mf5*IG_r*IG_r;
            d_ = df0+df1*IG_u+df2*IG_r+df3*IG_u*IG_u+df4*IG_u*IG_r+df5*IG_r*IG_r;
            zone = 0;
            if((m<=0.5*d) || (m<=-0.5*d)){
                if(IG_r>=0){
                    //Zona Azul
                    m = mr0+mr1*IG_u+mr2*IG_r+mr3*IG_u*IG_u+mr4*IG_u*IG_r+mr5*IG_r*IG_r;
                    d_ = dr0+dr1*IG_u+dr2*IG_r+dr3*IG_u*IG_u+dr4*IG_u*IG_r+dr5*IG_r*IG_r;
                    zone = 1;
                }else{
                    //Zona Verde
                    m = mr0+mr1*IG_u-mr2*IG_r+mr3*IG_u*IG_u-mr4*IG_u*IG_r+mr5*IG_r*IG_r;
                    d_ = dr0-dr1*IG_u+dr2*IG_r-dr3*IG_u*IG_u+dr4*IG_u*IG_r-dr5*IG_r*IG_r;
                    zone = -1;
                }
            }
        }

        double L, R;
        L = ((2 * m + d_) / 2);
        R = ((2 * m - d_) / 2);

        if (L > 0) {
            L = L + Dz_2;
        } else if (L < 0){
            L = L + Dz_1;
        }

        if (R > 0) {
            R = R + Dz_2;;
        } else if (R < 0){
            R = R + Dz_1;
        }
        
        // Publish pwms
        msg.t_left=400 * L + 1500;
        msg.t_righ=400 * R + 1500;
        if(msg.t_left<1100){
            msg.t_left=1100;
        }else if (msg.t_left > 1900) {
            msg.t_left=1900;
        }
        if(msg.t_righ<1100){
            msg.t_righ=1100;
        }else if (msg.t_righ > 1900) {
            msg.t_righ=1900;
        }
        msg_Ig.x = IG_u;
        msg_Ig.y = IG_r;
        msg_Ig.z = zone;
        publisher_pwm->publish(msg);
        publisher_IG->publish(msg_Ig);


    }else{
        msg.t_left= 1500;
        msg.t_righ= 1500; 
        count=count+1;
        publisher_pwm->publish(msg);
        publisher_IG->publish(msg_Ig);
    }
        

    }
}


//Coordinate Transformation   
    Eigen::Vector3d CoordinateTransformation(Eigen::Vector<double, 6>& X_est) {
        double x_bar = X_est(0) + eps_i * std::cos(X_est(2));
        double y_bar = X_est(1) + eps_i * std::sin(X_est(2));
        double v_bar = X_est(4) + eps_i * X_est(5);
        Eigen::Vector3d res;
        res.setZero();
        res << x_bar, y_bar, v_bar;
        return res;
      }

//Beta Function
    Eigen::Vector4d betaFunction(double t) {
        Eigen::Vector2d lim_cn;
        lim_cn.setZero();
        lim_cn << d_cn, theta_cn;
        Eigen::Vector2d lim_cl;
        lim_cl.setZero();
        lim_cl << d_cl, theta_cl;
        Eigen::Vector2d K;
        K.setZero();
        K << Kd, Ktheta;
        Eigen::Vector2d binf;
        binf.setZero();
        binf << b_dinf, b_thetainf;
        Eigen::Vector2d ref;
        ref.setZero();
        ref << ref_d, ref_theta;
        
        // Calcola b0 = lim_cn - ref
        Eigen::Vector2d b0 = lim_cn - ref;
        
        // Calcola beta_d e beta_theta
        double beta_d     = (b0(0) - binf(0)) * std::exp(-K(0) * t) + binf(0);
        double beta_theta = (b0(1) - binf(1)) * std::exp(-K(1) * t) + binf(1);
        
        // Calcola le derivate beta_ddot e beta_thetadot
        double beta_ddot     = -K(0) * (b0(0) - binf(0)) * std::exp(-K(0) * t);
        double beta_thetadot = -K(1) * (b0(1) - binf(1)) * std::exp(-K(1) * t);
        
        // Restituisce un vettore a 4 elementi
        Eigen::Vector4d out;
        out.setZero();
        out << beta_d, beta_theta, beta_ddot, beta_thetadot;
        return out;
    }    

    //High gain observer
    Eigen::Vector2d HighGainObserver(const Eigen::Vector3d &Xl_bar) {
        if (isFirstStep_HGO) {
          // Initial Condition
          eps1_0 = Eigen::Vector2d::Zero();
          eps2_0 = Eigen::Vector2d::Zero();
          isFirstStep_HGO = false;
        } else {
          Eigen::Vector2d eps1 = eps1_0;
          Eigen::Vector2d eps2 = eps2_0;
          
         //Euler method
          double dx1 = eps2(0);
          double dx2 = eps2(1);
          double dx3 = -lambda * eps2(0) - eps1(0) + Xl_bar(0);
          double dx4 = -lambda * eps2(1) - eps1(1) + Xl_bar(1);
          
          eps1(0) += dx1 * Ts;
          eps1(1) += dx2 * Ts;
          eps2(0) += dx3 * Ts;
          eps2(1) += dx4 * Ts;
        
          eps1_0 = eps1;
          eps2_0 = eps2;
          //RCLCPP_INFO(this->get_logger(), "eps2_0(0): %.5f and eps2_0(1): %.5f", eps2_0(0), eps2_0(1));
        }
    
        return eps2_0 / zeta;
      }

    //Compute errorHG
    double ComputeErrorHG(const Eigen::Vector4d &beta, double d, double e_d, double e_theta) {
        Eigen::Vector2d q = Compute_q(beta, e_d, e_theta);
    
        if (isFirstStep_EHG) {
          h_hat = 0.0;
          isFirstStep_EHG = false;
        } else {
          double dh = gamma * ( std::abs(e_d * q(0)) + (1.0 / d) * std::abs(e_theta * q(1)) - sigma_h * h_hat );
          h_hat += Ts * dh;
        }
        return h_hat;
      }

    //Compute q
    Eigen::Vector2d Compute_q(const Eigen::Vector4d &beta, double e_d, double e_theta) {
        Eigen::Vector2d q;
        q(0) = pow(1.0 / cos((M_PI * pow(e_d, 2)) / (2 * pow(beta(0), 2))), 2);
        q(1) = pow(1.0 / cos((M_PI * pow(e_theta, 2)) / (2 * pow(beta(1), 2))), 2);
        return q;
    }
    
    //DSC
    Eigen::Vector2d DSC(double alpha_ui, double alpha_ri,
        double e_d, double e_theta,
        const Eigen::Vector2d &q, double theta)
    {
        if (isFirstStep_DSC) {
        alpha_fui = alpha_ui + mu_u * e_d * q(0) * std::cos(theta);
        alpha_fri = alpha_ri + mu_r * e_theta * q(1);
        isFirstStep_DSC = false;
        } else {
        double alpha_fu = alpha_fui;
        double alpha_fr = alpha_fri;

        double alpha_mu = alpha_ui + mu_u * e_d * q(0) * std::cos(theta);
        double alpha_mr = alpha_ri + mu_r * e_theta * q(1);

        alpha_fu = alpha_fu + (Ts / mu_u) * (alpha_mu - alpha_fu);
        alpha_fr = alpha_fr + (Ts / mu_r) * (alpha_mr - alpha_fr);

        alpha_fui = alpha_fu;
        alpha_fri = alpha_fr;
        }

        Eigen::Vector2d out;
        out.setZero();
        out << alpha_fui, alpha_fri;
        return out;
    }
    //
    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat = msg->point.x;
            y_hat = msg->point.y;
            psi_hat = msg->point.z;
            u_hat = msg->velocity.x;
            v_hat = msg->velocity.y;
            r_hat = msg->velocity.z;
            sig_u = msg->disturbances.x;
            sig_v = msg->disturbances.y;
            sig_r = msg->disturbances.z;
        }
    
    void callbackNeighbor(const asv_interfaces::msg::StateNeighbor::SharedPtr msg)
       {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat_l = msg->point.x;
            y_hat_l = msg->point.y;
            psi_hat_l = msg->point.z;
            u_hat_l = msg->velocity.x;
            v_hat_l = msg->velocity.y;
            r_hat_l = msg->velocity.z;
        }
    
        void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
        {
            armed= msg->armed;
            // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
        }
    
    bool armed = true;
    int count = 0;
    float x_hat = 0, y_hat = 0, u_hat = 0, v_hat = 0, r_hat = 0, psi_hat = 0, sig_u = 0, sig_v = 0, sig_r = 0;
    float x_hat_l = 0, y_hat_l = 0, u_hat_l = 0, v_hat_l = 0, r_hat_l = 0, psi_hat_l = 0;
    double d = 0.0, theta = 0.0;
    double d_cn = 0.0, theta_cn = 0.0, d_cl = 0.0, theta_cl = 0.0;
    double Kd = 0.0, Ktheta = 0.0, b_dinf = 0.0, b_thetainf = 0.0;
    double ref_d = 0.0, ref_theta = 0.0;
    float eps_i = 0.0;
    double taud = 0.0;
    double Ts = 0.0;      
    double lambda = 0.0;  
    double zeta = 0.0;
    double gamma = 0.0;
    double sigma_h = 0.0;
    double mu_u = 0.0;    
    double mu_r = 0.0;    
    double m11 = 0.0, m33_bar = 0.0, m22 = 0.0;
    double k_d = 0.0, k_theta = 0.0;
    double k_u = 0.0, k_r = 0.0;
    double Sigma = 0.0;

    float mf0, mf1, mf2, mf3, mf4, mf5;
    float mr0, mr1, mr2, mr3, mr4, mr5;
    float df0, df1, df2, df3, df4, df5;
    float dr0, dr1, dr2, dr3, dr4, dr5;

    Eigen::Vector2d R1;
    Eigen::Vector2d R2;

    std::vector<double> memory_u;
    std::vector<double> memory_r;

    Eigen::Vector2d eps1_0;
    Eigen::Vector2d eps2_0;

    bool isFirstStep_HGO = true;
    bool isFirstStep_EHG = true;
    double h_hat = 0.0;
    bool isFirstStep_DSC = true;
  
    double alpha_fui = 0.0;
    double alpha_fri = 0.0;
    float IGumax_ff, IGumax_rf, IGumin_rf;
    float IGrmax_ff, IGrmax_rf;
    float Dz_1, Dz_2, p, q;


    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateNeighbor>::SharedPtr subscriber_state_neighbor_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_IG;

    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LyapHlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
