#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/reference_llc.hpp"

#include <pluginlib/class_loader.hpp>
#include "acados_solver_base/acados_solver.hpp"
#include "acados_solver_base/acados_solver_utils.hpp"

#include <cmath>
#include <vector>
#include <array>

using std::placeholders::_1;
using Refsize = std::array<double, 8>;

class MpcLlcRtNode : public rclcpp::Node 
{
public:
    MpcLlcRtNode() : Node("mpc_llc_rt")
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);

        this->declare_parameter<int>("N_p", 30);
        this->declare_parameter<float>("T_mlc", 400.0);
        this->declare_parameter<std::vector<double>>("Q", {6.0, 0.1, 30.0, 60.0});
        this->declare_parameter<std::vector<double>>("R", {0.0, 0.0});
        this->declare_parameter<std::vector<double>>("Rd", {10.0, 0.1});

        this->declare_parameter<double>("u_max", 3.0);
        this->declare_parameter<double>("u_min", 0.0);
        this->declare_parameter<double>("v_max", 1.5);
        this->declare_parameter<double>("v_min", -1.5);
        this->declare_parameter<double>("r_max", 1.0);
        this->declare_parameter<double>("r_min", -1.0);
        this->declare_parameter<double>("Delta_mean_min", -1.0);
        this->declare_parameter<double>("Delta_diff_min", -1.0);
        this->declare_parameter<double>("Delta_mean_max", 1.0);
        this->declare_parameter<double>("Delta_diff_max", 1.0);

        this-> declare_parameter("Xu", std::vector<double>{-0.2888875, -0.5552270, -0.2058061, -0.4044921, -0.2059754, 9.8420940});
        this-> declare_parameter("Xv", std::vector<double>{-0.2569846, 0.0294050, -0.0309902, 0.1500647, -0.0083264, 0.7489566, -0.2673529, 0.2186005, 0.3292047, -0.1586187, -0.2921791, -1.1262066});
        this-> declare_parameter("Xr", std::vector<double>{0.0265067, -0.2475340, 0.1360967, -0.5179457, 0.0066795, -0.0971898, 0.1288800, -0.9505331, -1.3230426, 0.6374735, 1.1742405, 4.5261182});
        this-> declare_parameter("Dz_up", 0.0001);
        this-> declare_parameter("Dz_down", -0.0001);

        Ts = this->get_parameter("Ts").as_double()/1000.0;

        my_id = (this->get_parameter("my_id").as_string());

        // Leer parámetros y asignar a variables miembro
        this->get_parameter("N_p", N_p);
        Q_ = this->get_parameter("Q").as_double_array();
        R_ = this->get_parameter("R").as_double_array();
        Rd_ = this->get_parameter("Rd").as_double_array();

        this->get_parameter("u_max", u_max_);
        this->get_parameter("u_min", u_min_);
        this->get_parameter("v_max", v_max_);
        this->get_parameter("v_min", v_min_);
        this->get_parameter("r_max", r_max_);
        this->get_parameter("r_min", r_min_);
        this->get_parameter("Delta_mean_min", Delta_mean_min_);
        this->get_parameter("Delta_diff_min", Delta_diff_min_);
        this->get_parameter("Delta_mean_max", Delta_mean_max_);
        this->get_parameter("Delta_diff_max", Delta_diff_max_);

        Xu = this->get_parameter("Xu").as_double_array();
        Xv = this->get_parameter("Xv").as_double_array();
        Xr = this->get_parameter("Xr").as_double_array();
        Dz_up  = this->get_parameter("Dz_up").as_double();
        Dz_down = this->get_parameter("Dz_down").as_double();

        T_mlc = this->get_parameter("T_mlc").as_double()/1000.0;
        N_r = static_cast<int>(T_mlc / Ts);

        std::string solver_plugin_name = "asv_acados/AsvAcadosSolver";
        acados_solver_loader_ = std::make_shared<pluginlib::ClassLoader<acados::AcadosSolver>>("acados_solver_base", "acados::AcadosSolver");
        acados_solver_ = std::unique_ptr<acados::AcadosSolver>(acados_solver_loader_->createUnmanagedInstance(solver_plugin_name));
        std::cout << "Loading solver plugin \"" << solver_plugin_name << "\"" << std::endl;

        Precompile();
        init_hist();

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->get_node_base_interface()->get_default_callback_group();
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcLlcRtNode::calculateLowLevelController, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcLlcRtNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&MpcLlcRtNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<asv_interfaces::msg::ReferenceLlc>(
            "/" + my_id + "/control/reference_llc", 1, std::bind(&MpcLlcRtNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&MpcLlcRtNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/" + my_id + "/control/pwm_value_mpc",
                1);
        publisher_mps_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/mpc_state_llc",1);
        
    	RCLCPP_INFO(this->get_logger(), "Mpc Llc Real Time Node has been started.");
    }

private:

    void calculateLowLevelController()
    {
        bool armed_loc;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed_loc= armed;
        }
        if(armed_loc==false){
            count=0;
            mean_ant   = 0.0;
            diff_ant   = 0.0;
            d_mean_ant = 0.0;
            d_diff_ant = 0.0;
            is_first_itr_ = true;

            init_hist();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat = 0.0;
                v_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;
                sig_u = 0.0;
                sig_v = 0.0;
                sig_r = 0.0;
                flag_ref = false;
                flag_iter = false;
            }
        }else{
            auto msg = asv_interfaces::msg::PwmValues();
            if(count > 8){
                auto start = std::chrono::high_resolution_clock::now();
                double u_hat_i;
                double v_hat_i;
                double r_hat_i;
                double psi_hat_i;
                double sig_u_i;
                double sig_v_i;
                double sig_r_i;
                std::vector<Refsize> y_ref_i;
                y_ref_i.reserve(static_cast<size_t>(N_p));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    u_hat_i= u_hat;
                    v_hat_i= v_hat;
                    r_hat_i=r_hat;
                    psi_hat_i=psi_hat;
                    sig_u_i=sig_u;
                    sig_v_i=sig_v;
                    sig_r_i=sig_r;
                    if(!flag_ref){
                        initRefereces_unsafe(psi_hat_i);
                    }    
                    if (flag_iter) {
                        y_ref_i.assign(y_refs_.begin(), y_refs_.end());
                        flag_iter = false;
                    } else {
                        y_ref_i.assign(y_refs_.begin() + 1, y_refs_.end());
                        y_ref_i.push_back(y_refs_.back());
                    }      
                }

                for (int j = 0; j < N_p; j++){
                    for (int i = 0; i < 8; ++i) {
                        yref_buf_[i] = y_ref_i[j][static_cast<size_t>(i)];
                    }
                    if (true != acados::utils::set_cost_y_ref(*acados_solver_, j, yref_buf_)) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC references!");
                    } 
                    if(j<N_p-1){
                        if (0 != acados_solver_->initialize_control_values(j, u_hist[j+1])) {
                            RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC control!");
                        }  
                    }else{
                        if (0 != acados_solver_->initialize_control_values(j, u_hist[j])) {
                            RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC control!");
                        } 
                    }
                }
                for (int i = 0; i < 6; ++i) {
                    yref_e_buf_[i] = y_ref_i.back()[static_cast<size_t>(i)];
                }
                if (true != acados::utils::set_cost_y_ref(*acados_solver_, N_p, yref_e_buf_)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC final reference!");
                }   
                
                x_values_map["u"][0]    = u_hat_i;
                x_values_map["v"][0]    = v_hat_i;
                x_values_map["r"][0]    = r_hat_i;
                x_values_map["psi"][0]  = psi_hat_i;
                x_values_map["mean"][0] = mean_ant;
                x_values_map["diff"][0] = diff_ant;

                // acados::ValueVector x_values;
                // acados::AcadosSolver::fill_vector_from_map(acados_solver_->x_index_map(), x_values_map, 6, x_values);

                if (is_first_itr_) {  // this is the first iteration
                    // Set initial state values for all stages of the NMPC problem
                    (0 == acados_solver_->initialize_state_values(x_values_map));
                    // Update the first iteration flag
                    is_first_itr_ = false;
                }
                //Set initial state values for the first stage of the NMPC problem
                (0 == acados_solver_->set_initial_state_values(x_values_map));               

                p_values_map["se_u"][0] = sig_u_i;
                p_values_map["se_v"][0] = sig_v_i;
                p_values_map["se_r"][0] = sig_r_i;

                if (0 != acados_solver_->set_runtime_parameters(p_values_map)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC runtime parameters!");
                }

                double j_min;
                int n_iter;
                double t_proc;

                // Solve NMPC optimization problem
                int solver_status = acados_solver_->solve();
                if (0 != solver_status) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to solve the NMPC SQP problem, status = != %i",solver_status);
                    j_min = -1;
                    n_iter = 0;
                    t_proc = 0;
                    d_mean_ant = u_hist[1]["d_mean"][0];
                    d_diff_ant = u_hist[1]["d_diff"][0];
                    
                    mean_ant = mean_ant + Ts*d_mean_ant;
                    diff_ant = diff_ant + Ts*d_diff_ant;

                    // RCLCPP_ERROR(this->get_logger(), "D_mean: %f y D_diff: %f",d_mean_ant, d_diff_ant);
                    std::move(u_hist.begin() + 1, u_hist.end(), u_hist.begin());
                    u_hist.back() = u_hist[u_hist.size() - 2];   // copia del penúltimo al último

                    Precompile();
                } else {
                    // Get optimal control input
                    for (int k = 0; k < N_p; ++k) {
                        acados::ValueMap u_values_map = acados_solver_->get_control_values_as_map(k);
                        if (k==0)
                        {
                            d_mean_ant = u_values_map["d_mean"][0];
                            d_diff_ant = u_values_map["d_diff"][0];
                        }
                        u_hist[k] = u_values_map; 
                    }                    
                    //RCLCPP_ERROR(this->get_logger(), "Solucion viable D_mean: %f y D_diff: %f",d_mean_ant, d_diff_ant);

                    mean_ant = mean_ant + Ts*d_mean_ant;
                    diff_ant = diff_ant + Ts*d_diff_ant;

                    //RCLCPP_ERROR(this->get_logger(), "Solucion viable mean: %f y diff: %f",mean_ant, diff_ant);

                    /*acados::ValueMap un_values_map = acados_solver_->get_state_values_as_map(1);
                    double mean_1 = un_values_map["mean"][0];
                    double diff_1 = un_values_map["diff"][0];
                    RCLCPP_ERROR(this->get_logger(), "Solucion calculada mean: %f y diff: %f",mean_1, diff_1);*/

                    j_min = acados::utils::get_stats_cost_value(*acados_solver_);
                    n_iter = acados::utils::get_stats_sqp_iter(*acados_solver_);
                    //t_proc = acados::utils::get_stats_cpu_time(*acados_solver_);
                }

                double m = mean_ant; 
                double d = diff_ant;

                double L, R;
                L = ((2 * m + d) / 2);
                R = ((2 * m - d) / 2);

                if (L > 0) {
                    L = L + Dz_up;
                } else if (L < 0){
                    L = L + Dz_down;
                }

                if (R > 0) {
                    R = R + Dz_up;;
                } else if (R < 0){
                    R = R + Dz_down;
                }

                // Publish pwms
                msg.t_left = denormalizationPwm(L);
                msg.t_righ = denormalizationPwm(R);
                publisher_pwm->publish(msg);

                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                t_proc = elapsed.count()*1000; // miliseconds

                auto msg_s = geometry_msgs::msg::Vector3();
                msg_s.x = j_min;
                msg_s.y = n_iter;
                msg_s.z = t_proc;
                publisher_mps_state->publish(msg_s);
            }else{
                msg.t_left= 1500;
                msg.t_righ= 1500; 
                count=count+1;
                publisher_pwm->publish(msg);
            }
        }        
    }

    uint16_t denormalizationPwm(double delta) {
        int resultado = static_cast<int>(400 * delta) + 1500;
        if (resultado < 1100) {
            resultado = 1100;
        } else if (resultado > 1900) {
            resultado = 1900;
        }
        return static_cast<uint16_t>(resultado);
    }

    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_hat = msg->velocity.x;
            v_hat = msg->velocity.y;
            r_hat = msg->velocity.z;
            psi_hat = msg->point.z;
            sig_u = msg->disturbances.x;
            sig_v = msg->disturbances.y;
            sig_r = msg->disturbances.z;
        }
    }

    void callbackVelReference(const asv_interfaces::msg::ReferenceLlc::SharedPtr msg){
        int Np_i, Nr_i;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Np_i = N_p;
            Nr_i = N_r; 
        }

        const auto &refs = msg->references;
        const std::size_t n_total = refs.size() * static_cast<std::size_t>(Nr_i);
        const std::size_t limit = std::min<std::size_t>(static_cast<std::size_t>(Np_i), n_total);

        std::vector<Refsize> out;
        out.reserve(Np_i);

        auto make_ref8 = [](const geometry_msgs::msg::Vector3 &v) -> Refsize {
            return Refsize{{static_cast<double>(v.x), 0.0, static_cast<double>(v.y), static_cast<double>(v.z), 0.0, 0.0, 0.0, 0.0
            }};
        };

        if (refs.size() == 1) {
            const auto r = make_ref8(refs[0]);
            for (int i = 0; i < Np_i; ++i) {
                out.push_back(r);
            }
        } else {
            std::size_t produced = 0;
            for (const auto &v : refs) {
                const auto r = make_ref8(v);
                for (int k = 0; k < Nr_i; ++k) {
                    if (produced >= limit) break;
                    out.push_back(r);
                    ++produced;
                }
                if (produced >= limit) break;
            }
        }

        if (out.size() < static_cast<size_t>(Np_i)) {
            out.resize(static_cast<size_t>(Np_i), out.back());
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            y_refs_.swap(out);
            flag_ref = true;
            flag_iter = true;
        }
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed= msg->armed;
        }
    }

    void initRefereces_unsafe(float psi_act){
        std::vector<Refsize> out;
        out.reserve(static_cast<size_t>(N_p));

        const double psi = static_cast<double>(psi_act);

        for (int i = 0; i < N_p; ++i) {
            out.push_back(Refsize{{
                0.5, 0.0, 0.0, psi,
                0.0, 0.0, 0.0, 0.0
            }});
        }

        y_refs_.swap(out);
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

        for (const auto &param: params){
            if (param.get_name() == "Ts") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 100.0 && param.as_double() <= 1000.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Ts = param.as_double()/1000.0;
                    if (timer_) {
                        timer_->cancel();
                    }
                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(int(Ts*1000.0)),
                        std::bind(&MpcLlcRtNode::calculateLowLevelController, this),
                        cb_group_obs_);
                    N_r = static_cast<int>(T_mlc / Ts);
                    need_precompile = true;
                }
                else {
                    return reject("could not change param Ts", "Ts: double in [100,1000] ms");
                }
            }

            if (param.get_name() == "T_mlc") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 100.0 && param.as_double() <= 1000.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    T_mlc = param.as_double()/1000.0;
                    N_r = static_cast<int>(T_mlc / Ts);
                } else {
                    return reject("could not change param T_mlc", "T_mlc: double in [100,1000] ms");
                }
            }

            if (param.get_name() == "N_p") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                    param.as_int() >= 2 && param.as_int() <= 40) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_p = param.as_int();
                    need_precompile = true;
                } else {
                    return reject("could not change param N_p", "N_p: integer in [2,40]");
                }
            }

            if (param.get_name() == "Q") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
                    param.as_double_array().size() == 4) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Q_ = param.as_double_array();
                    need_precompile = true;
                } else {
                    return reject("could not change param Q", "Q: double array with 4 elements");
                }
            }

            if (param.get_name() == "R") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
                    param.as_double_array().size() == 2) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    R_ = param.as_double_array();
                    need_precompile = true;
                } else {
                    return reject("could not change param R", "R: double array with 2 elements");
                }
            }

            if (param.get_name() == "Rd") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY &&
                    param.as_double_array().size() == 2) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Rd_ = param.as_double_array();
                    need_precompile = true;
                } else {
                    return reject("could not change param Rd", "Rd: double array with 2 elements");
                }
            }

            if (param.get_name() == "Delta_mean_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -1.0 && param.as_double() < 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_mean_min_ = param.as_double();
                    need_precompile = true;
                } else {
                    return reject("could not change param Delta_mean_min", "Delta_mean_min: double in [-1.0,0.0)");
                }
            }

            if (param.get_name() == "Delta_diff_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -2.0 && param.as_double() < 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_diff_min_ = param.as_double();
                    need_precompile = true;
                } else {
                    return reject("could not change param Delta_diff_min", "Delta_diff_min: double in [-2.0,0.0)");
                }
            }

            if (param.get_name() == "Delta_mean_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() <= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_mean_max_ = param.as_double();
                    need_precompile = true;
                } else {
                    return reject("could not change param Delta_mean_max", "Delta_mean_max: double in (0.0,1.0]");
                }
            }

            if (param.get_name() == "Delta_diff_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() <= 2.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_diff_max_ = param.as_double();
                    need_precompile = true;
                } else {
                    return reject("could not change param Delta_diff_max", "Delta_diff_max: double in (0.0,2.0]");
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

     /*-----------------------------------------------MPC Funtions---------------------------------------------------*/

    void Precompile(){
        RCLCPP_INFO(this->get_logger(), "Initializing Acados solver with N = %i and Ts = %f", N_p, Ts);
        acados_solver_->init(N_p, Ts);

        x_values_map["u"]    = std::vector<double>{0.0};
        x_values_map["v"]    = std::vector<double>{0.0};
        x_values_map["r"]    = std::vector<double>{0.0};
        x_values_map["psi"]  = std::vector<double>{0.0};
        x_values_map["mean"] = std::vector<double>{0.0};
        x_values_map["diff"] = std::vector<double>{0.0};

        p_values_map["Xu"] = Xu;
        p_values_map["Xv"] = Xv;
        p_values_map["Xr"] = Xr;
        p_values_map["Dz_up"] = std::vector{static_cast<double>(Dz_up)};
        p_values_map["Dz_down"] = std::vector{static_cast<double>(Dz_down)};
        p_values_map["se_u"] = std::vector{0.0};
        p_values_map["se_v"] = std::vector{0.0};
        p_values_map["se_r"] = std::vector{0.0};

        if (0 != acados_solver_->set_runtime_parameters(p_values_map)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC runtime parameters!");
        }

        acados::IndexVector idxbu = {0,1};
        acados::ValueVector lbu = {Delta_mean_min_/Ts, Delta_diff_min_/Ts};
        acados::ValueVector ubu = {Delta_mean_max_/Ts, Delta_diff_max_/Ts};

        acados::IndexVector idxbx = {0,1,2};
        acados::ValueVector lbx = {u_min_, v_min_, r_min_};
        acados::ValueVector ubx = {u_max_, v_max_, r_max_};

        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(Q_.size() + R_.size() + Rd_.size(),
                                                  Q_.size() + R_.size()+ Rd_.size());
        W.diagonal().head(Q_.size()) = Eigen::Map<const Eigen::VectorXd>(Q_.data(), Q_.size());
        W.diagonal().segment(Q_.size(), R_.size()) = Eigen::Map<const Eigen::VectorXd>(R_.data(),  R_.size());
        W.diagonal().tail(Rd_.size()) = Eigen::Map<const Eigen::VectorXd>(Rd_.data(), Rd_.size());

        Eigen::MatrixXd We = Eigen::MatrixXd::Zero(Q_.size() + R_.size(),
                                                   Q_.size() + R_.size());
        We.diagonal().head(Q_.size()) = Eigen::Map<const Eigen::VectorXd>(Q_.data(), Q_.size());
        We.diagonal().tail(R_.size()) = Eigen::Map<const Eigen::VectorXd>(R_.data(), R_.size());

        Eigen::VectorXd h_min(5);
        h_min << -1e9, -1.0 - Dz_down, -1e9, -1.0 - Dz_down, 0.0;

        Eigen::VectorXd h_max(5);
        h_max << 1.0 - Dz_up, 1e9, 1.0 - Dz_up, 1e9, 1e9;

        u_hist.resize(static_cast<size_t>(N_p));   // crea N_p elementos

        for (int idx = 0; idx <= N_p; idx++) {
            if (idx>0){
                acados_solver_->set_state_bounds(idx, idxbx, lbx, ubx);
            } if (idx==N_p){
                acados::utils::set_cost_W(*acados_solver_, N_p, We);
            } if (idx < N_p){
                acados_solver_->set_control_bounds(idx, idxbu, lbu, ubu);
                acados::utils::set_cost_W(*acados_solver_, idx, W);
            }  if (idx>0 && idx <N_p){
                acados::utils::set_const_h_min(*acados_solver_, idx, h_min);
                acados::utils::set_const_h_max(*acados_solver_, idx, h_max);
            }
        }
    }

    void init_hist(){
        acados::ValueMap u_def;
        u_def["d_mean"] = {0.001 / Ts};
        u_def["d_diff"] = std::vector{0.0};
        for (int k = 0; k < N_p; ++k) {
            u_hist[k] = u_def; 
        }
    }

    bool armed = false, flag_ref = false, is_first_itr_= true, flag_iter = true;
    double u_hat = 0.0, v_hat = 0, psi_hat = 0, r_hat = 0, sig_u = 0, sig_v = 0, sig_r = 0;

    double mean_ant = 0.0, diff_ant = 0.0, d_mean_ant= 0.0, d_diff_ant=0.0;
    int count=0;
    //------Params-------//
    float Ts, T_mlc;  

    int N_p, N_r;
    double u_max_, u_min_, v_max_, v_min_, r_max_, r_min_, Delta_mean_min_, Delta_diff_min_, Delta_mean_max_, Delta_diff_max_;
    double Dz_up, Dz_down;  

    std::vector<double> Xu, Xv, Xr;
    std::vector<double> Q_, R_, Rd_;

    std::vector<Refsize> y_refs_;
    Eigen::VectorXd yref_buf_{8};
    Eigen::VectorXd yref_e_buf_{6};

    std::vector<acados::ValueMap> u_hist;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::ReferenceLlc>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_mps_state;
    rclcpp::TimerBase::SharedPtr timer_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    /// Acados solver pluginlib loader
    std::shared_ptr<pluginlib::ClassLoader<acados::AcadosSolver>> acados_solver_loader_;
    /// Acados solver
    std::unique_ptr<acados::AcadosSolver> acados_solver_;
    acados::ValueMap p_values_map;
    acados::ValueMap x_values_map;

};