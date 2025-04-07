#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"  
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <mutex>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

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
        this->declare_parameter("Ts", 0.01);
        this->declare_parameter("lambda", 1.0);
        this->declare_parameter("zeta", 1.0);
        this->declare_parameter("gamma", 1.0);
        this->declare_parameter("sigma_h", 1.0);
        this->declare_parameter("mu_u", 1.0);
        this->declare_parameter("mu_r", 1.0);
        this->declare_parameter("k_d", 1.0);
        this->declare_parameter("k_theta", 1.0);
        this->declare_parameter("m11", 1.0);
        this->declare_parameter("m33_bar", 1.0);
        this->declare_parameter("m22", 1.0);
        this->declare_parameter("k_u", 1.0);
        this->declare_parameter("k_r", 1.0);
        this->declare_parameter("Sigma", 1.0);
        this->declare_parameter("d_cn", 10.0);
        this->declare_parameter("theta_cn", 1.0);
        this->declare_parameter("d_cl", 5.0);
        this->declare_parameter("theta_cl", 0.5);
        this->declare_parameter("Kd", 1.0);
        this->declare_parameter("Ktheta", 1.0);
        this->declare_parameter("b_dinf", 0.1);
        this->declare_parameter("b_thetainf", 0.1);
        this->declare_parameter("ref_d", 0.0);
        this->declare_parameter("ref_theta", 0.0);
        this->declare_parameter("eps_i", 0.1);

        // Get parameters
        my_id = this->get_parameter("my_id").as_string();
        
        taud = this->get_parameter("taud").as_double();
        Ts = this->get_parameter("Ts").as_double();
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
        
        }

private:
    void calculateHighLevelController()
    {
        
        if(armed==false){
            count = 0;
            // TODO: reiniciar datos que varian con el tiempo
        }else{
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

        Eigen::VectorXd Xf_est(6);
        Xf_est << x_hat_i, y_hat_i, psi_hat_i, u_hat_i, v_hat_i, r_hat_i;
        Eigen::VectorXd Xl_est(6);
        // (Xl_est << 1.5, 2.5, 0.9, 0.0, 3.5, 0.3;)
        Xl_est << x_hat_l_i, y_hat_l_i, psi_hat_l_i, u_hat_l_i, v_hat_l_i, r_hat_l_i;
        Eigen::Vector3d sigmaf_est;
        sigmaf_est << sig_u_i, sig_v_i, sig_r_i;
        double t = this->now().seconds(); 

        // --- Rotational matrices --- (From Simulator to paper)
        Eigen::Matrix3d Rs2pg, Rs2pb;
        Rs2pg << 0, 1, 0,
                 1, 0, 0,
                 0, 0, -1;
        Rs2pb << 1, 0, 0,
                 0, -1, 0,
                 0, 0, -1;

        Xf_est.segment(0, 3) = Rs2pg * Xf_est.segment(0, 3);
        Xf_est.segment(3, 3) = Rs2pb * Xf_est.segment(3, 3);
        Xl_est.segment(0, 3) = Rs2pg * Xl_est.segment(0, 3);
        Xl_est.segment(3, 3) = Rs2pb * Xl_est.segment(3, 3);
        sigmaf_est = Rs2pb * sigmaf_est;

        // --- Coordinate Transformation ---
        Eigen::Vector3d Xf_bar = CoordinateTransformation(Xf_est);
        Eigen::Vector3d Xl_bar = CoordinateTransformation(Xl_est);

        double xf = Xf_bar(0);
        double yf = Xf_bar(1);
        
        double psi = Xf_est(2);
        double xl = Xl_bar(0);
        double yl = Xl_bar(1);

        double e1 = std::cos(psi) * (xl - xf) + std::sin(psi) * (yl - yf);
        double e2 = -std::sin(psi) * (xl - xf) + std::cos(psi) * (yl - yf);
        double d = std::sqrt(std::pow(xl - xf, 2) + std::pow(yl - yf, 2));
        double theta = std::atan2(e2, e1);

        Eigen::Vector2d ref;
        ref << d, theta;

        double e_d = d - ref_d;
        double e_theta = theta - ref_theta;

        Eigen::Vector4d beta = betaFunction(t);

        Eigen::Vector2d p_dot_l_est = HighGainObserver(Xl_bar);

        Eigen::VectorXd q = Compute_q(beta, e_d, e_theta); 

        double H_hat = ComputeErrorHG(beta, d, e_d, e_theta);

        double var1_u = std::pow(beta(0), 2) / (2 * M_PI * e_d);
        double var2_u = (2 * beta(2) / beta(0)) + k_d;
        double var3_u = std::sin(M_PI * std::pow(e_d, 2)) / std::pow(beta(0), 2);
        double var4_u = (beta(2) * e_d) / beta(0);
        double var5_u = Xf_bar(2) * std::sin(theta);
        double var6_u = p_dot_l_est.dot(R1);
        double var7_u = H_hat * std::tanh(e_d * q(0) * H_hat / Sigma);

        double var1_r = std::pow(beta(1), 2) / (2 * M_PI * e_theta);
        double var2_r = (2 * beta(3) / beta(1)) + k_theta;
        double var3_r = std::sin(M_PI * std::pow(e_theta, 2) / std::pow(beta(1), 2));
        double var4_r = (beta(3) * e_theta) / beta(1);
        
        double var5_r = Xf_est(3) * std::sin(theta);
        double var6_r = Xf_bar(2) * std::cos(theta);
        double var7_r = p_dot_l_est.dot(R2);
        double var8_r = (H_hat / d) * std::tanh(e_theta * q(1) * H_hat / (Sigma * d));

        double alpha_ui = (1 / std::cos(theta)) * (var1_u * var2_u * var3_u - var4_u - var5_u + var6_u + var7_u);
        double alpha_ri = var1_r * var2_r * var3_r - var4_r + (var5_r - var6_r + var7_r) / d + var8_r;

        //TODO: cuidado alpha no se está utilizando
        Eigen::Vector2d alpha = Eigen::Vector2d(alpha_ui, alpha_ri);
        Eigen::Vector2d alpha_f = DSC(alpha, e_d, e_theta, q, theta);

        double e21 = Xf_est(3) - alpha_f(0); 
        double e22 = Xf_est(5) - alpha_f(1); 
        Eigen::Vector2d e2i;
        e2i << e21, e22;
        double e_alpha1 = alpha_f(0) - alpha_ui;
        double e_alpha2 = alpha_f(1) - alpha_ri;
        Eigen::Vector2d e_alpha;
        e_alpha << e_alpha1, e_alpha2;

        double tau_u = m11 * (-k_u * e21 - e_alpha1 / mu_u + 2 * e_d * q(0) * std::cos(theta)) - sigmaf_est(0);
        double tau_r = (m33_bar / m22) * (-k_r * e22 - e_alpha2 / mu_r + 2 * e_theta * q(1)) - sigmaf_est(2);

        Eigen::Vector3d Tau_real;
        Tau_real << tau_u, 0, tau_r;
        
        Tau_real = Rs2pb.transpose() * Tau_real;
        // Aquí se publicaría Tau_real usando el publisher (si estuviera habilitado)

        RCLCPP_INFO(this->get_logger(), "Tau_real: %f, %f, %f", Tau_real(0), Tau_real(1), Tau_real(2));
        }
        else{
            Eigen::Vector3d Tau_real;
            Tau_real << 0, 0, 0;
        RCLCPP_INFO(this->get_logger(), "Tau_real: %f, %f, %f", Tau_real(0), Tau_real(1), Tau_real(2));

            count++;
        }
        }
        // el siguiente paso es pasar de Tau a PWM
    }

//Coordinate Transformation   
    Eigen::Vector3d CoordinateTransformation(Eigen::VectorXd& X_est) {
        double x_bar = X_est[0] + eps_i * std::cos(X_est[2]);
        double y_bar = X_est[1] + eps_i * std::sin(X_est[2]);
        double v_bar = X_est[4] + eps_i * X_est[5];
        Eigen::Vector3d res;
        res << x_bar, y_bar, v_bar;
        return res;
      }

//Beta Function
    Eigen::Vector4d betaFunction(double t) {
        Eigen::Vector2d lim_cn;
        lim_cn << d_cn, theta_cn;
        Eigen::Vector2d lim_cl;
        lim_cl << d_cl, theta_cl;
        Eigen::Vector2d K;
        K << Kd, Ktheta;
        Eigen::Vector2d binf;
        binf << b_dinf, b_thetainf;
        Eigen::Vector2d ref;
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
        }
    
        return eps2_0 / zeta;
      }

    //Compute errorHG
    double ComputeErrorHG(const Eigen::VectorXd &beta, double d, double e_d, double e_theta) {
        Eigen::VectorXd q = Compute_q(beta, e_d, e_theta);
    
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
    Eigen::VectorXd Compute_q(const Eigen::VectorXd &beta, double e_d, double e_theta) {
        Eigen::VectorXd q(2);
        q(0) = pow(1.0 / cos((M_PI * pow(e_d, 2)) / (2 * pow(beta(0), 2))), 2);
        q(1) = pow(1.0 / cos((M_PI * pow(e_theta, 2)) / (2 * pow(beta(1), 2))), 2);
        return q;
    }
    
    //DSC
    Eigen::Vector2d DSC(const Eigen::Vector2d& alpha,
        double e_d, double e_theta,
        const Eigen::VectorXd &q, double theta)
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
    
        void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
        {
            armed= msg->armed;
            // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
        }
    
    bool armed = false;
    int count = 0;
    float x_hat = 0, y_hat = 0, u_hat = 0, v_hat = 0, r_hat = 0, psi_hat = 0, sig_u = 0, sig_v = 0, sig_r = 0;
    float x_hat_l = 0, y_hat_l = 0, u_hat_l = 0, v_hat_l = 0, r_hat_l = 0, psi_hat_l = 0;
    
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

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
   //rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;

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
