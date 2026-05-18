#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/reference_llc.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"      //Interface velocity

#include <pluginlib/class_loader.hpp>
#include "acados_solver_base/acados_solver.hpp"
#include "acados_solver_base/acados_solver_utils.hpp"
// #include "asv_library/curvas_sim.h"
#include "asv_library/curvas_alamillo.h"

#include <cmath>
#include <vector>

using std::placeholders::_1;


class IblosmpMlcPfNode : public rclcpp::Node 
{
public:
    IblosmpMlcPfNode() : Node("iblosmp_mlc_pf")
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //---------Parámetros del PF-------------------//
        this-> declare_parameter("Ts", 200.0);

        this->declare_parameter<int>("N_p", 60);
        this->declare_parameter<double>("T_p", 3.0);

        this-> declare_parameter("delta_LOS", 8.0);
        this-> declare_parameter("k_u_tar", 2.0);
        this-> declare_parameter("k_b", 0.01);
        this-> declare_parameter("k_psi", 0.1);
        this-> declare_parameter("taud", 15.0); // Taud = #*Ts Est es #

        this->declare_parameter<double>("u_ref_max", 3.0);
        this->declare_parameter<double>("u_ref_min", 0.3);
        this->declare_parameter<double>("u_tar_max", 3.0);
        this->declare_parameter<double>("u_tar_min", 0.0);
        this->declare_parameter<double>("r_ref_max", 0.6);
        this->declare_parameter<double>("r_ref_min", -0.6);
        this->declare_parameter<double>("Delta_u_ref_min", -0.5);
        this->declare_parameter<double>("Delta_u_ref_max", 0.5);
        this->declare_parameter<double>("Delta_r_ref_min", -0.5);
        this->declare_parameter<double>("Delta_r_ref_max", 0.5);
        this->declare_parameter("beta_bar_dot_max", 0.03);

        this-> declare_parameter("Xv_bar", std::vector<double>{-0.2503897, -0.0321815, 0.0028707, 0.0211998, -0.0066645, 0.7247758, -0.2352875, -0.0178921});
        this-> declare_parameter<double>("Eps", 0.2488);

        this-> declare_parameter("path_d", 0); // path_d = #Path deseado #

        // Leer parámetros y asignar a variables miembro
        
        my_id = (this->get_parameter("my_id").as_string());    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        
        this->get_parameter("N_p", N_p);
        delta_LOS = this->get_parameter("delta_LOS").as_double();
        k_u_tar = this->get_parameter("k_u_tar").as_double();
        k_b = this->get_parameter("k_b").as_double();
        k_psi = this->get_parameter("k_psi").as_double();
        taud_ = this->get_parameter("taud").as_double();

        this->get_parameter("u_ref_max", u_ref_max_);
        this->get_parameter("u_ref_min", u_ref_min_);
        this->get_parameter("u_tar_max", u_tar_max_);
        this->get_parameter("u_tar_min", u_tar_min_);
        this->get_parameter("r_ref_max", r_ref_max_);
        this->get_parameter("r_ref_min", r_ref_min_);
        this->get_parameter("Delta_u_ref_min", Delta_u_ref_min_);
        this->get_parameter("Delta_u_ref_max", Delta_u_ref_max_);
        this->get_parameter("Delta_r_ref_min", Delta_r_ref_min_);
        this->get_parameter("Delta_r_ref_max", Delta_r_ref_max_);
        
        beta_bar_dot_max = this->get_parameter("beta_bar_dot_max").as_double();

        Xv_bar = this->get_parameter("Xv_bar").as_double_array();
        this->get_parameter("Eps", Eps_);

        path_d  = this->get_parameter("path_d").as_int();

        this->get_parameter("T_p", T_p);
        N_t = static_cast<int>(std::ceil(T_p / Ts));

        double denom = Ts * (taud_ + 1.0);
        a = (taud_ * Ts) / denom;
        b = 1.0 / denom;

        std::string solver_plugin_name = "asv_acados/IgblosAcados";
        acados_sim_loader_ = std::make_shared<pluginlib::ClassLoader<acados::AcadosSolver>>("acados_solver_base", "acados::AcadosSolver");
        acados_sim_ = std::unique_ptr<acados::AcadosSolver>(acados_sim_loader_->createUnmanagedInstance(solver_plugin_name));
        std::cout << "Loading solver plugin \"" << solver_plugin_name << "\"" << std::endl;

        Precompile();

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->get_node_base_interface()->get_default_callback_group();
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&IblosmpMlcPfNode::param_callback, this, _1));

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&IblosmpMlcPfNode::calculateMidLevelController, this), cb_group_obs_);

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&IblosmpMlcPfNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_mlc", 1, std::bind(&IblosmpMlcPfNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&IblosmpMlcPfNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        /*subscriber_vel_ = this-> create_subscription<geometry_msgs::msg::TwistStamped>("/" + my_id + "/mavros/local_position/velocity_body",
                rclcpp::SensorDataQoS(), std::bind(&IblosmpMlcPfNode::callbackVel, this, std::placeholders::_1), options_sensors_); */ 

        publisher_llc = this-> create_publisher<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1);
        publisher_error = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/error_mlc",1);
        publisher_mpc_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/mpc_state_mlc",1);
        publisher_los_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/los_state_mlc",1);
        
    	RCLCPP_INFO(this->get_logger(), "IBLOS MP MLC Path Following Real Time Node has been started.");
    }

private:

    void calculateMidLevelController()
    {
        bool armed_loc;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed_loc= armed;
        }
        if(armed_loc==false){
            count=0;
            w_i = 0;
            beta_bar = 0;
            v_bar_ff = 0;
            psi_d_1_ant = 0;
            psi_d_dot1_ant = 0;
            u_ref_ant = 0.0;
            r_ref_ant = 0.0;
            armed_act=false;
            laps = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat = 0.0;
                v_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;
                u_tar_ant = 0.0;
                u_d = 0.8;
            }
        }else{
            auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::ReferenceLlc();
            auto msg_e = geometry_msgs::msg::Vector3();
            auto msg_los = geometry_msgs::msg::Vector3();
            if(count > 4){
                double x_hat_i;
                double y_hat_i;
                double u_hat_i;
                double v_hat_i;
                double r_hat_i;
                double psi_hat_i;
                double u_d_i;

                double xe_bar_i;
                double ye_bar_i;
                double v_bar_i;
                double xe_i;
                double ye_i;

                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    x_hat_i = x_hat;
                    y_hat_i = y_hat;
                    u_hat_i = u_hat;
                    v_hat_i = v_hat;
                    r_hat_i = r_hat;
                    psi_hat_i=psi_hat;
                    u_d_i=u_d;
                }

                get_inicial_values(x_hat_i, y_hat_i, psi_hat_i, v_hat_i, r_hat_i, xe_bar_i, ye_bar_i, v_bar_i, xe_i, ye_i);  


                acados::ValueMap x_values_map;
                x_values_map["x_e_bar"] = std::vector{xe_bar_i};
                x_values_map["y_e_bar"] = std::vector{ye_bar_i};
                x_values_map["psi"] =  std::vector{psi_hat_i};
                x_values_map["w"] = std::vector{w_i};
                x_values_map["v_bar"] = std::vector{v_bar_i};
                
                acados::ValueMap x_next_map = x_values_map;
                acados::ValueVector x_values;
                acados::ValueVector x_next_values;
                acados::AcadosSolver::fill_vector_from_map(acados_sim_->x_index_map(), x_values_map, 5, x_values);
                acados::AcadosSolver::fill_vector_from_map(acados_sim_->x_index_map(), x_next_map, 5, x_next_values);

                acados::ValueMap u_values_map;
                acados::ValueVector u_values;
                u_values_map["u_ref"] = std::vector{0.4};
                u_values_map["u_tar"] = std::vector{0.0};
                u_values_map["r_ref"] =  std::vector{0.0};
                acados::AcadosSolver::fill_vector_from_map(acados_sim_->u_index_map(), u_values_map, 3, u_values);

                double beta_bar_j = beta_bar;
                double v_bar_ff_j = v_bar_ff;
                u_values[0] = u_hat_i;
                u_values[1] = u_tar_ant;
                u_values[2] = r_hat_i;
                double psi_d_dot1_ant_j = psi_d_dot1_ant;
                double psi_d_1_ant_j = psi_d_1_ant;
                con_hist.clear();
                psi_ref_hist.clear();
                
                for (int k = 0; k < N_p; ++k) {

                    Target p_j = currentTarget(x_values[3]);
                    const double phip_j = p_j.phip;
                    const double dphip_j = p_j.dphip;
                    const double Fp_j = p_j.f_c;

                    const double xe_j = x_values[0] - Eps_* cos(x_values[2] - phip_j);
                    const double ye_j = x_values[1] - Eps_* sin(x_values[2] - phip_j);
                    const double v_hat_j = x_values[4] - Eps_*u_values[2];

                    const double beta_hat_i = atan2(v_hat_j,u_values[0]);
                    const double chi_hat =  x_values[2] + beta_hat_i;

                    const double k1_i = u_d_i / delta_LOS;
                    double u_ref = k1_i * std::sqrt(delta_LOS*delta_LOS + ye_j*ye_j);
                    double U_hat = std::sqrt(u_values[0]*u_values[0] + v_hat_j*v_hat_j);
                    double u_tar = k_u_tar*xe_j + U_hat*cos(chi_hat-phip_j);
                    u_tar = std::clamp(u_tar, u_tar_min_, u_tar_max_);
                    const double w_dot = u_tar / Fp_j;

                    double chi_d = phip_j - std::atan2(ye_j, delta_LOS);

                    const double chi_e = std::atan2(std::sin(chi_hat - chi_d), std::cos(chi_hat - chi_d));
                    double beta_bar_dot = k_b * chi_e;
                    beta_bar_dot = std::clamp(beta_bar_dot, -beta_bar_dot_max, beta_bar_dot_max);
                    beta_bar_j += beta_bar_dot * Ts;

                    const double chi_d_dot = dphip_j * w_dot;
                    const double dot_v_bar_ff = Xv_bar[5]*u_d_i * chi_d_dot + Xv_bar[6] * v_bar_ff_j + (Xv_bar[7]-Eps_*Xv_bar[6])*chi_d_dot;
                    v_bar_ff_j += dot_v_bar_ff * Ts;
                    const double v_ff = v_bar_ff_j - Eps_*chi_d_dot;
                    const double beta_ff = std::atan2(v_ff, u_d_i);

                    const double beta_est = beta_bar_j + beta_ff;
                    double psi_ref = chi_d - beta_est;
                    if(psi_ref<0){
                        psi_ref=psi_ref+(2*M_PI);
                    }

                    const double psi_d_1 = - beta_est - atan2(ye_j,delta_LOS);
                    const double psi_d_dot1 = (a * psi_d_dot1_ant_j) + b * atan2(sin(psi_d_1 - psi_d_1_ant_j), cos(psi_d_1 - psi_d_1_ant_j));
                    const double psi_dot_ref = chi_d_dot + psi_d_dot1;
                    double r_ref = psi_dot_ref  - k_psi * atan2(sin(x_values[2] - psi_ref), cos(x_values[2] - psi_ref));

                    psi_d_1_ant_j = psi_d_1;
                    psi_d_dot1_ant_j = psi_d_dot1;

                    r_ref = std::clamp(r_ref, r_ref_min_, r_ref_max_);   // clamp(x, min, max)
                    double dr_ref = r_ref - r_ref_ant;
                    dr_ref = std::clamp(dr_ref, Delta_r_ref_min_, Delta_r_ref_max_);
                    r_ref  = r_ref_ant + dr_ref;

                    // --- u ---
                    u_ref = std::clamp(u_ref, u_ref_min_, u_ref_max_);
                    double du_ref = u_ref - u_ref_ant;
                    du_ref = std::clamp(du_ref, Delta_u_ref_min_, Delta_u_ref_max_);
                    u_ref  = u_ref_ant + du_ref;

                    u_values[0] = u_ref;
                    u_values[1] = u_tar;
                    u_values[2] = r_ref;
                    //RCLCPP_INFO(this->get_logger(), "r_sat: %.5f and beta", r_ref);

                    int sim_status = acados_sim_->simulate(Ts, x_values, u_values, p_values, x_next_values, z_values);

                    if (sim_status != 0) {
                        RCLCPP_INFO(this->get_logger(), "Error in 'AcadosSolver::simulate()': Simulation failed");
                    }

                    //

                    con_hist.push_back(u_values);
                    psi_ref_hist.push_back(psi_ref);

                    if(k==0){
                        beta_bar = beta_bar_j;
                        v_bar_ff = v_bar_ff_j;
                        psi_d_1_ant = psi_d_1_ant_j;
                        psi_d_dot1_ant = psi_d_dot1_ant_j;
                        u_tar_ant = u_tar;
                        
                        msg_los.x = beta_bar;
                        msg_los.y = beta_ff;
                        msg_los.z = beta_bar_dot;
                    }
                    r_ref_ant = r_ref;
                    u_ref_ant = u_ref;

                    // Ahora x_values pasa a ser el nuevo estado sin copiar memoria
                    std::swap(x_values, x_next_values);
                }

                // RCLCPP_INFO(this->get_logger(), "dot: %.5f and beta %.5f", beta_bar_dot, beta_bar);

                msg.references.clear();
                msg.references.reserve(static_cast<size_t>(N_t));

                unwrapAngleSeqInPlace(psi_ref_hist);

                if(armed_act==false){
                    psi_ant = psi_ref_hist[0];
                    laps = 0;
                }else{
                    if((psi_ref_hist[0] - psi_ant) > M_PI){
                        laps = laps - 1;
                    }else if((psi_ref_hist[0] - psi_ant) < -M_PI){
                        laps = laps + 1;
                    }
                    psi_ant=psi_ref_hist[0];
                }

                for (int i = 0; i < N_t; i++)
                {
                    const int idx = (i < N_p) ? i : N_p - 1;
                    if(i<N_p){
                        auto ref_i = geometry_msgs::msg::Vector3();
                        if(idx == 0){
                            u_ref_ant = con_hist[idx][0];
                            r_ref_ant = con_hist[idx][2];
                        }
                        ref_i.x = con_hist[idx][0];
                        ref_i.y = con_hist[idx][2];
                        ref_i.z = psi_ref_hist[idx] + 2*M_PI*laps ;
                        msg.references.push_back(ref_i);
                    } 
                }

                msg.u_tar.data = u_tar_ant;

                msg_e.x = xe_i;
                msg_e.y = ye_i;
                msg_e.z = w_i;

                publisher_llc->publish(msg);
                publisher_error->publish(msg_e);
                publisher_los_state->publish(msg_los);

                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                double t_proc = elapsed.count()*1000; // miliseconds

                // Misma variable distinto topico
                msg_e.x = 0.0;
                msg_e.y = 0.0;
                msg_e.z = t_proc;
                publisher_mpc_state->publish(msg_e);     
                armed_act=true;           
            }else{
                count=count+1;
            }
        }        
    }

    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat = msg->point.x;
            y_hat = msg->point.y;
            psi_hat = msg->point.z;
            u_hat = msg->velocity.x;
            v_hat = msg->velocity.y;
            r_hat = msg->velocity.z;
        }
    }

    void callbackVelReference(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_d = msg->data;
        }
    }


    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed= msg->armed;
        }
    }

    void get_inicial_values(double x, double y, double psi, double v, double r,
                        double &xe_bar, double &ye_bar, double &v_bar, double &xe, double &ye) {

        evolve_w();

        Target p_i = currentTarget(w_i);

        // Obtener puntos de trayectoria
        double xp = p_i.xp;
        double yp = p_i.yp;
        double phip = p_i.phip;

        // Matriz de rotación transpuesta
        double R11 =  cos(phip);
        double R12 =  sin(phip);
        double R21 = -sin(phip);
        double R22 =  cos(phip);

        // Vector auxiliar
        xe_bar = R11*(x + Eps_*cos(psi) - xp) + R12*(y + Eps_*sin(psi) - yp);
        ye_bar = R21*(x + Eps_*cos(psi) - xp) + R22*(y + Eps_*sin(psi) - yp);

        // xe = xe_bar - Eps_*cos(psi-phip);
        // ye = ye_bar - Eps_*sin(psi-phip);
        xe = R11*(x - xp) + R12*(y - yp);
        ye = R21*(x - xp) + R22*(y - yp);
        v_bar = v + Eps_*r;
    }

    void evolve_w() {
        Target p_i = currentTarget(w_i);
        double w_dot = u_tar_ant / p_i.f_c;
        w_i += Ts * w_dot;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        bool armed_local;
        bool need_precompile = false;

        auto reject = [&](const std::string &log_msg, const std::string &reason_msg) {
            RCLCPP_INFO(this->get_logger(), "%s", log_msg.c_str());
            result.successful = false;
            result.reason = reason_msg;
            return result;
        };
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed_local = armed;
        }
        if (armed_local) {
            return reject("could not change params", "ARMED: parameter changes blocked");
        }

        for (const auto &param : params) {
            if (param.get_name() == "Ts") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 100.0 && param.as_double() < 1000.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Ts = param.as_double() / 1000.0;

                    if (timer_) {
                        timer_->cancel();
                    }
                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(int(Ts * 1000.0)),
                        std::bind(&IblosmpMlcPfNode::calculateMidLevelController, this),
                        cb_group_obs_);

                    N_t = static_cast<int>(std::ceil(T_p / Ts));

                    double denom = Ts * (taud_ + 1.0);
                    a = (taud_ * Ts) / denom;
                    b = 1.0 / denom;

                    need_precompile = true;
                } else {
                    return reject("could not change param Ts",
                                "Ts: double in [100,1000) ms");
                }
            }
            if (param.get_name() == "N_p") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                    param.as_int() >= 2 && param.as_int() <= 100) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_p = param.as_int();
                    need_precompile = true;
                } else {
                    return reject("could not change param N_p",
                                "N_p: integer in [2,100]");
                }
            }
            if (param.get_name() == "T_p") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() < 5.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    T_p = param.as_double();
                    N_t = static_cast<int>(std::ceil(T_p / Ts));
                } else {
                    return reject("could not change param T_p",
                                "T_p: double in (0,5) s");
                }
            }
            if (param.get_name() == "delta_LOS") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    delta_LOS = param.as_double();
                } else {
                    return reject("could not change param delta_LOS",
                                "delta_LOS: double in (0,100)");
                }
            }
            if (param.get_name() == "k_u_tar") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    k_u_tar = param.as_double();
                } else {
                    return reject("could not change param k_u_tar",
                                "k_u_tar: double in (0,100)");
                }
            }
            if (param.get_name() == "k_b") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    std::isfinite(param.as_double()) &&
                    param.as_double() >= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    k_b = param.as_double();
                } else {
                    return reject("could not change param k_b",
                                "k_b: finite double >= 0");
                }
            }
            if (param.get_name() == "k_psi") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    std::isfinite(param.as_double()) &&
                    param.as_double() >= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    k_psi = param.as_double();
                } else {
                    return reject("could not change param k_psi",
                                "k_psi: finite double >= 0");
                }
            }
            if (param.get_name() == "taud") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 500.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    taud_ = param.as_double();

                    double denom = Ts * (taud_ + 1.0);
                    a = (taud_ * Ts) / denom;
                    b = 1.0 / denom;
                } else {
                    return reject("could not change param taud",
                                "taud: double in [0,500)");
                }
            }
            if (param.get_name() == "r_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() <= 0.6) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    r_ref_max_ = param.as_double();
                } else {
                    return reject("could not change param r_ref_max",
                                "r_ref_max: double in (0.0,0.6]");
                }
            }
            if (param.get_name() == "r_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -0.6 && param.as_double() < 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    r_ref_min_ = param.as_double();
                } else {
                    return reject("could not change param r_ref_min",
                                "r_ref_min: double in [-0.6,0.0)");
                }
            }
            if (param.get_name() == "Delta_u_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -0.5 && param.as_double() < 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_u_ref_min_ = param.as_double();
                } else {
                    return reject("could not change param Delta_u_ref_min",
                                "Delta_u_ref_min: double in [-0.5,0.0)");
                }
            }
            if (param.get_name() == "Delta_u_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() <= 0.5) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_u_ref_max_ = param.as_double();
                } else {
                    return reject("could not change param Delta_u_ref_max",
                                "Delta_u_ref_max: double in (0.0,0.5]");
                }
            }
            if (param.get_name() == "Delta_r_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -0.5 && param.as_double() < 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_r_ref_min_ = param.as_double();
                } else {
                    return reject("could not change param Delta_r_ref_min",
                                "Delta_r_ref_min: double in [-0.5,0.0)");
                }
            }
            if (param.get_name() == "Delta_r_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() <= 0.5) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_r_ref_max_ = param.as_double();
                } else {
                    return reject("could not change param Delta_r_ref_max",
                                "Delta_r_ref_max: double in (0.0,0.5]");
                }
            }
            if (param.get_name() == "beta_bar_dot_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    std::isfinite(param.as_double()) &&
                    param.as_double() > 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    beta_bar_dot_max = param.as_double();
                } else {
                    return reject("could not change param beta_bar_dot_max",
                                "beta_bar_dot_max: finite double > 0");
                }
            }
            if (param.get_name() == "path_d") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                    param.as_int() >= 0 && param.as_int() <= 5) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    path_d = param.as_int();
                    need_precompile = true;
                } else {
                    return reject("could not change param path_d",
                                "path_d: integer in [0,5]");
                }
            }
        }
        if (need_precompile) {
            Precompile();
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    Target currentTarget(double w){
        Target result;
        switch(path_d) {
            case 0:
                result.xp = w+10;
                result.yp = 10;
                result.dxp = 1;
                result.dyp = 0;
                result.phip = 0;
                result.dphip = 0;
                result.f_c = 1;
                break;
            case 1:
                result = line_south(w);
                // result = line_northeast(w);
                break;
            case 2:
                result = circle_10m(w);
                // result = circle_30m(w);
                break;
            case 3:
                result = circle_8m(w);
                break;
            case 4:
                result = lissajous_10m(w);
                break;
            case 5:
                result = lissajous_5m(w);
                break;
        }
        return result;
    }

    static inline void unwrapAngleSeqInPlace(std::vector<double>& psi){
        if (psi.size() < 2) return;

        constexpr double TWO_PI = 6.2831853071795864769;

        double prev_wrapped   = psi[0];
        double prev_unwrapped = psi[0];

        for (std::size_t i = 1; i < psi.size(); ++i)
        {
            double d = psi[i] - prev_wrapped;     // salto entre muestras envueltas
            d = std::remainder(d, TWO_PI);        // lo lleva a (-pi, pi]
            prev_unwrapped += d;                  // acumula cambio continuo

            prev_wrapped = psi[i];
            psi[i] = prev_unwrapped;
        }
    }

     /*-----------------------------------------------MPC Funtions---------------------------------------------------*/

    void Precompile(){
        RCLCPP_INFO(this->get_logger(), "Initializing Acados IGBLOS solver with N = %i and Ts = %f", N_p, Ts);
        acados_sim_->init(N_p, Ts);
        acados::ValueMap p_values_map;
        p_values_map["Xv_bar"] = Xv_bar;
        p_values_map["Eps_"] = std::vector{Eps_};

        switch(path_d) {
            case 0:
                p_values_map["coef"] = std::vector{0.0, 0.0, 1.0, 0.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
            case 1:
                // p_values_map["coef"] = std::vector{0.0, 0.0, 1.0, 1.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                p_values_map["coef"] = std::vector{0.0, 0.0,-1.0, 0.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
            case 2:
                // p_values_map["coef"] = std::vector{-30.0, 30.0, 0.0, 0.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                p_values_map["coef"] = std::vector{10.0, -10.0, 0.0, 0.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
            case 3:
                p_values_map["coef"] = std::vector{8.0, -8.0, 0.0, 0.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
            case 4:
                p_values_map["coef"] = std::vector{0.0, 0.0, 0.0, 0.0, 10.0, 15.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
            case 5:
                p_values_map["coef"] = std::vector{0.0, 0.0, 0.0, 0.0, 5.0, 15.0};  //  a y b Circulo c y d Lineal e y f lissa
                break;
        }

        acados::AcadosSolver::fill_vector_from_map(acados_sim_->p_index_map(), p_values_map, 15, p_values);

        acados::ValueMap z_values_map;
        z_values_map["x_e"] = std::vector{0.0};
        z_values_map["y_e"] = std::vector{0.0};

        acados::AcadosSolver::fill_vector_from_map(acados_sim_->z_index_map(), z_values_map, 2, z_values);

        if (0 != acados_sim_->set_runtime_parameters(p_values_map)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC runtime parameters!");
        }

        acados::ValueMap x_values_map;
        x_values_map["x_e_bar"] = std::vector{0.0};
        x_values_map["y_e_bar"] = std::vector{0.8};
        x_values_map["psi"] =  std::vector{0.0};
        x_values_map["w"] = std::vector{2.0};
        x_values_map["v_bar"] = std::vector{0.0};
        
        acados::ValueMap x_next_map = x_values_map;
        acados::ValueVector x_values;
        acados::ValueVector x_next_values;
        acados::AcadosSolver::fill_vector_from_map(acados_sim_->x_index_map(), x_values_map, 5, x_values);

        con_hist.reserve(static_cast<size_t>(N_p));
        psi_ref_hist.reserve(static_cast<size_t>(N_p));


    }


    bool armed = false, armed_act=false;
    double psi_hat = 0, r_hat = 0, u_hat=0, v_hat = 0, x_hat = 0, y_hat = 0, u_d = 0.8;
    double w_i = 0.0;

    double beta_bar = 0,  v_bar_ff = 0, psi_d_1_ant = 0, psi_d_dot1_ant = 0, psi_ant = 0;

    double u_tar_ant = 0.0, u_ref_ant = 0.0, r_ref_ant = 0.0;
    int count=0, laps = 0;
    //------Params-------//
    double Ts, T_p;

    int N_p, N_t;
    double u_ref_max_, u_ref_min_, u_tar_max_, u_tar_min_;
    double r_ref_max_, r_ref_min_, Delta_u_ref_min_, Delta_u_ref_max_, Delta_r_ref_min_, Delta_r_ref_max_;
    double Eps_;
    double taud_;
    double beta_bar_dot_max;

    double delta_LOS; /*Ganancia delta SGLOS*/
    double k_u_tar; /*Ganancia de la velocidad de surge target*/
    double k_b; /*Ganancia Integral*/
    double k_psi; /*Ganancia velocidad de giro*/
    double a ,b; /*Constantes del filtro derivativo*/

    int path_d; /*Variable para elegir path*/

    std::vector<double> Xv_bar;

    std::vector<acados::ValueVector> con_hist;
    std::vector<double> psi_ref_hist;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::ReferenceLlc>::SharedPtr publisher_llc;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_error;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_mpc_state;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_los_state;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_vel_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    /// Acados solver pluginlib loader
    std::shared_ptr<pluginlib::ClassLoader<acados::AcadosSolver>> acados_sim_loader_;
    std::unique_ptr<acados::AcadosSolver> acados_sim_;
    acados::ValueVector p_values, z_values;
};