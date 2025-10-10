#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/reference_llc.hpp"     //Interface reference_llc references y utar
#include "std_msgs/msg/float32_multi_array.hpp"     //Interface w predictions
#include "std_msgs/msg/float32.hpp"                 // w

#include "geometry_msgs/msg/twist_stamped.hpp"      //Interface velocity

#include <pluginlib/class_loader.hpp>
#include "acados_solver_base/acados_solver.hpp"
#include "acados_solver_base/acados_solver_utils.hpp"

#include <cmath>
#include <thread>
#include <vector>
#include <Eigen/Dense>

using std::placeholders::_1;

class MpcMlcPfCoorRtNode : public rclcpp::Node 
{
public:
    MpcMlcPfCoorRtNode() : Node("mpc_mlc_pf_coor_rt")
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //---------Parámetros del PF-------------------//
        this-> declare_parameter("Ts", 400.0);

        this->declare_parameter<int>("N_p", 60);
        this->declare_parameter<int>("N_w", 20);
        this->declare_parameter<float>("T_p", 3.0);
        this->declare_parameter<std::vector<double>>("Q", {10.0, 10.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("R", {1.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("Rd", {0.0, 0.0, 0.0});
        this->declare_parameter<double>("Dw", 5.0);

        this->declare_parameter<double>("x_e_bar_max", 100.0);
        this->declare_parameter<double>("x_e_bar_min", -100.0);
        this->declare_parameter<double>("y_e_bar_max", 100.0);
        this->declare_parameter<double>("y_e_bar_min", -100.0);
        this->declare_parameter<double>("v_bar_max", 2.0);
        this->declare_parameter<double>("v_bar_min", -2.0);
        this->declare_parameter<double>("u_ref_max", 3.0);
        this->declare_parameter<double>("u_ref_min", 0.3);
        this->declare_parameter<double>("u_tar_max", 3.0);
        this->declare_parameter<double>("u_tar_min", 0.5);
        this->declare_parameter<double>("r_ref_max", 0.6);
        this->declare_parameter<double>("r_ref_min", -0.6);
        this->declare_parameter<double>("Delta_u_ref_min", -0.5);
        this->declare_parameter<double>("Delta_u_ref_max", 0.5);
        this->declare_parameter<double>("Delta_u_tar_min", -0.5);
        this->declare_parameter<double>("Delta_u_tar_max", 0.5);
        this->declare_parameter<double>("Delta_r_ref_min", -0.5);
        this->declare_parameter<double>("Delta_r_ref_max", 0.5);

        this-> declare_parameter("Xv_bar", std::vector<double>{-0.2503897, -0.0321815, 0.0028707, 0.0211998, -0.0066645, 0.7247758, -0.2352875, -0.0178921});
        this-> declare_parameter<double>("Eps", 0.2488);

        this-> declare_parameter<double>("rho", 4.0);
        this-> declare_parameter<double>("theta", 0.0);

        this-> declare_parameter("path_d", 0); // path_d = #Path deseado #
        
        my_id = (this->get_parameter("my_id").as_string());    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        
        // Leer parámetros y asignar a variables miembro
        this->get_parameter("N_p", N_p);
        this->get_parameter("N_w", N_w);
        Q_ = this->get_parameter("Q").as_double_array();
        R_ = this->get_parameter("R").as_double_array();
        Rd_ = this->get_parameter("Rd").as_double_array();
        D_w_ = this->get_parameter("Dw").as_double();

        this->get_parameter("x_e_bar_max", x_e_bar_max_);
        this->get_parameter("x_e_bar_min", x_e_bar_min_);
        this->get_parameter("y_e_bar_max", y_e_bar_max_);
        this->get_parameter("y_e_bar_min", y_e_bar_min_);
        this->get_parameter("v_bar_max", v_bar_max_);
        this->get_parameter("v_bar_min", v_bar_min_);
        this->get_parameter("u_ref_max", u_ref_max_);
        this->get_parameter("u_ref_min", u_ref_min_);
        this->get_parameter("u_tar_max", u_tar_max_);
        this->get_parameter("u_tar_min", u_tar_min_);
        this->get_parameter("r_ref_max", r_ref_max_);
        this->get_parameter("r_ref_min", r_ref_min_);
        this->get_parameter("Delta_u_ref_min", Delta_u_ref_min_);
        this->get_parameter("Delta_u_ref_max", Delta_u_ref_max_);
        this->get_parameter("Delta_u_tar_min", Delta_u_tar_min_);
        this->get_parameter("Delta_u_tar_max", Delta_u_tar_max_);
        this->get_parameter("Delta_r_ref_min", Delta_r_ref_min_);
        this->get_parameter("Delta_r_ref_max", Delta_r_ref_max_);

        Xv_bar = this->get_parameter("Xv_bar").as_double_array();
        this->get_parameter("Eps", Eps_);

        rho  = this->get_parameter("rho").as_double();
        theta= this->get_parameter("theta").as_double();

        path_d= this->get_parameter("path_d").as_int();

        float T_p;
        this->get_parameter("T_p", T_p);
        N_t = static_cast<int>(std::ceil(T_p / Ts));

        std::string solver_plugin_name = "asv_acados/PfModAsvAcadosSolver";
        acados_solver_loader_ = std::make_shared<pluginlib::ClassLoader<acados::AcadosSolver>>("acados_solver_base", "acados::AcadosSolver");
        acados_solver_ = std::unique_ptr<acados::AcadosSolver>(acados_solver_loader_->createUnmanagedInstance(solver_plugin_name));
        std::cout << "Loading solver plugin \"" << solver_plugin_name << "\"" << std::endl;

        Precompile();

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcMlcPfCoorRtNode::param_callback, this, _1));

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcMlcPfCoorRtNode::calculateMidLevelController, this), cb_group_obs_);

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&MpcMlcPfCoorRtNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_mlc", 1, std::bind(&MpcMlcPfCoorRtNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&MpcMlcPfCoorRtNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        /*subscriber_vel_ = this-> create_subscription<geometry_msgs::msg::TwistStamped>("/" + my_id + "/mavros/local_position/velocity_body",
                rclcpp::SensorDataQoS(), std::bind(&MpcMlcPfCoorRtNode::callbackVel, this, std::placeholders::_1), options_sensors_); */ 

        publisher_llc = this-> create_publisher<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1);
        publisher_error = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/error_mlc",1);
        publisher_mpc_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/mpc_state_mlc",1);

        publisher_w_pre = this-> create_publisher<std_msgs::msg::Float32MultiArray>("/" + my_id + "/control/w_pred",1);

        // Lista de todos los barcos
        std::vector<std::string> all_ids = {"ASV0", "ASV1", "ASV3"};

        // Iteramos y nos suscribimos a todos menos a nosotros mismos
        for (const auto &id : all_ids) {
            if (id != my_id) {
                auto sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/" + id + "/control/w_pred", 1,
                    std::bind(&MpcMlcPfCoorRtNode::wPredCallback, this, std::placeholders::_1),// pasamos el id del barco también
                    options_sensors_);
                subs_.push_back(sub); // guardamos el shared_ptr en un vector para que no se destruya
            }
        }
        
    	RCLCPP_INFO(this->get_logger(), "Mpc MLC Path Following Coordinated Real Time Node has been started.");
    }

private:

    void calculateMidLevelController()
    {
        if(armed==false){
            count=0;
            w_i = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                v_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;

                u_hist.clear();
                u_hist.shrink_to_fit();
                acados::ValueMap u_def;
                u_def["d_u_ref"] = std::vector{0.0};
                u_def["d_u_tar"] = std::vector{0.0};
                u_def["d_r_ref"] = std::vector{0.0};

                con_hist.clear();
                con_hist.shrink_to_fit();
                acados::ValueMap con;
                con["u_ref"] = std::vector{0.5};
                con["u_tar"] = std::vector{0.4};
                con["r_ref"] = std::vector{0.0};

                w_avg.clear();
                w_avg.shrink_to_fit();
                w_avg.reserve(N_w);
                w_avg.emplace_back(w_i);
                
                // Get optimal control input
                for (int k = 0; k < N_p; ++k) {
                    u_hist.emplace_back(u_def); 
                    con_hist.emplace_back(con); 
                    if (k < N_w){
                        float w_k = w_avg.back();
                        evolve_w(w_k, 0.4);
                        w_avg.emplace_back(w_k);
                    }
                }   

                count_ws = 0;

                is_first_itr_= true; 
                u_ref_ant = 0.5;
                u_tar_ant = 0.5;
                r_ref_ant = 0.0;
                u_d = 0.8;
            }
        }else{
            auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::ReferenceLlc();
            auto msg_e = geometry_msgs::msg::Vector3();
            if(count > 2){
                float x_hat_i;
                float y_hat_i;
                float v_hat_i;
                float r_hat_i;
                float psi_hat_i;
                float u_d_i;

                float xe_i;
                float ye_i;
                float v_bar_i;
                std::vector<float> w_avg_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    x_hat_i = x_hat;
                    y_hat_i = y_hat;
                    v_hat_i = v_hat;
                    r_hat_i = r_hat;
                    psi_hat_i=psi_hat;
                    u_d_i=u_d;
                    w_avg_i=w_avg;
                    count_ws=0;
                }

                get_inicial_values(x_hat_i, y_hat_i, psi_hat_i, v_hat_i, r_hat_i, xe_i, ye_i, v_bar_i);                
                Eigen::VectorXd y_ref(11);
                for (int j = 0; j < N_p; j++){
                    
                    if(j < N_w){
                        y_ref << 0.0, 0.0, 0.0, w_avg_i[j], 0.0, 0.0, u_d_i, 0.0, 0.0, 0.0, 0.0;
                    }else{
                        y_ref << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, u_d_i, 0.0, 0.0, 0.0, 0.0;
                    }
                    if (true != acados::utils::set_cost_y_ref(*acados_solver_, j, y_ref)) {
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
                Eigen::VectorXd y_ref_e = y_ref.head(8);
                if (true != acados::utils::set_cost_y_ref(*acados_solver_, N_p, y_ref_e)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC final reference!");
                }   
                
                acados::ValueMap x_values_map;
                x_values_map["x_e_bar"] = std::vector{static_cast<double>(xe_i)};
                x_values_map["y_e_bar"] = std::vector{static_cast<double>(ye_i)};
                x_values_map["psi"] =  std::vector{static_cast<double>(psi_hat_i)};
                x_values_map["w"] = std::vector{static_cast<double>(w_i)};
                x_values_map["v_bar"] = std::vector{static_cast<double>(v_bar_i)};
                x_values_map["u_ref"] = std::vector{static_cast<double>(u_ref_ant)};
                x_values_map["u_tar"] = std::vector{static_cast<double>(u_tar_ant)};
                x_values_map["r_ref"] =  std::vector{static_cast<double>(r_ref_ant)};

                acados::ValueVector x_values;
                acados::AcadosSolver::fill_vector_from_map(acados_solver_->x_index_map(), x_values_map, 8, x_values);

                if (is_first_itr_) {  // this is the first iteration
                    // Set initial state values for all stages of the NMPC problem
                    (0 == acados_solver_->initialize_state_values(x_values));
                    // Update the first iteration flag
                    is_first_itr_ = false;
                }

                //Set initial state values for the first stage of the NMPC problem
                (0 == acados_solver_->set_initial_state_values(x_values));               

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

                    u_ref_ant = con_hist[1]["u_ref"][0];
                    u_tar_ant = con_hist[1]["u_tar"][0];
                    r_ref_ant = con_hist[1]["r_ref"][0];

                    const auto last = u_hist.back();                       // copia para duplicar
                    std::move(u_hist.begin() + 1, u_hist.end(), u_hist.begin()); // corre a la izquierda
                    u_hist.back() = last;                                  // último duplicado

                    const auto last_c = con_hist.back();                       // copia para duplicar
                    std::move(con_hist.begin() + 1, con_hist.end(), con_hist.begin()); // corre a la izquierda
                    con_hist.back() = last_c;                                  // último duplicado

                    Precompile();
                } else {
                    u_hist.clear();
                    u_hist.shrink_to_fit();
                    con_hist.clear();
                    con_hist.shrink_to_fit();
                    // Get optimal control input
                    for (int k = 0; k < N_p; ++k) {
                        acados::ValueMap xc_values_map = acados_solver_->get_state_values_as_map(k+1);
                        acados::ValueMap u_values_map = acados_solver_->get_control_values_as_map(k);
                        if (k==0)
                        {
                            u_ref_ant = xc_values_map["u_ref"][0];
                            u_tar_ant = xc_values_map["u_tar"][0];
                            r_ref_ant = xc_values_map["r_ref"][0];
                        }
                        u_hist.push_back(u_values_map); 
                        con_hist.push_back(xc_values_map); 
                    }                    

                    j_min = acados::utils::get_stats_cost_value(*acados_solver_);
                    n_iter = acados::utils::get_stats_sqp_iter(*acados_solver_);                    
                }

                float psi_i = psi_hat_i;

                std::vector<geometry_msgs::msg::Vector3> refs_;
                refs_.reserve(static_cast<size_t>(N_t));

                for (int i = 0; i < N_t; i++)
                {
                    auto msg_i = geometry_msgs::msg::Vector3();
                    msg_i.x = con_hist[i]["u_ref"][0];
                    msg_i.y = con_hist[i]["r_ref"][0];
                    msg_i.z = psi_i + Ts * msg_i.y;
                    psi_i = msg_i.z;
                    refs_.push_back(msg_i);
                }

                msg.references = refs_;
                msg.u_tar.data = u_tar_ant;

                // RCLCPP_INFO(this->get_logger(), "Valores Anteriores : %f , %f y %f", con_hist[0]["u_ref"][0], con_hist[0]["u_tar"][0], con_hist[0]["r_ref"][0]);

                msg_e.x = xe_i;
                msg_e.y = ye_i;
                msg_e.z = w_i;

                publisher_llc->publish(msg);
                publisher_error->publish(msg_e);

                std::vector<float> w_s;
                w_s.reserve(static_cast<size_t>(N_w));

                for (int i = 0; i < N_w; i++)
                {
                    w_s.push_back(con_hist[i]["w"][0]);
                }

                auto msg_ws = std_msgs::msg::Float32MultiArray();
                msg_ws.data = w_s;
                publisher_w_pre->publish(msg_ws);

                auto end = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed = end - start;
                t_proc = elapsed.count()*1000; // miliseconds

                // Misma variable distinto topico
                msg_e.x = j_min;
                msg_e.y = n_iter;
                msg_e.z = t_proc;
                publisher_mpc_state->publish(msg_e);                
            }else{
                count=count+1;
            }
        }        
    }

    void wPredCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg){
        std::vector<float> w_n;
        w_n.reserve(static_cast<size_t>(N_w));
        w_n = msg->data;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            count_ws += 1;
            float r1 = 1/count_ws;
            float r2 = 1 - r1;
            for (size_t i = 0; i < w_avg.size(); ++i) {
                w_avg[i] = w_avg[i] * r2 + w_n[i] * r1;
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
            v_hat = msg->velocity.y;
            r_hat = msg->velocity.z;
        }
    }

    /*void callbackVel(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            v_hat = msg->twist.linear.y;
            r_hat = msg->twist.angular.z;
        }
    }*/

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

    void get_inicial_values(float x, float y, float psi, float v, float r,
                        float &xe, float &ye, float &v_bar) {

        evolve_w(w_i, u_tar_ant);

        std::vector<float> path = spatial_path(w_i);

        // Obtener puntos de trayectoria
        float xc = path[0];
        float yc = path[1];
        float phic = path[2];
        float dxc = path[3];
        float dyc = path[4];
        float dphic = path[5];

        float xp = xc + rho * cos(phic+theta);  
        float yp = yc + rho * sin(phic+theta);
        float phip = atan2 (dyc + dphic*(rho * cos(phic+theta)), dxc + dphic*(rho * sin(phic+theta)));

        // Matriz de rotación transpuesta
        float R11 =  cos(phip);
        float R12 =  sin(phip);
        float R21 = -sin(phip);
        float R22 =  cos(phip);

        // Vector auxiliar
        float aux1 = R11*(x + Eps_*cos(psi) - xp) + R12*(y + Eps_*sin(psi) - yp);
        float aux2 = R21*(x + Eps_*cos(psi) - xp) + R22*(y + Eps_*sin(psi) - yp);

        xe = aux1;
        ye = aux2;
        v_bar = v + Eps_*r;
        // RCLCPP_INFO(this->get_logger(), "Target point: Xe: %f, Ye: %f y v_bar: %f ", xe, ye, v_bar);
    }

    void evolve_w(float &w, const float u_tar) {
        std::vector<float> path = spatial_path(w);
        float dx = path[3];
        float dy = path[4];
        float F = sqrt(dx*dx + dy*dy); // Norma
        float w_dot = u_tar / F;
        w += Ts * w_dot;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "N_p") {
                if (param.as_int() > 1 && param.as_int() < 100) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_p = param.as_int();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 1-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "N_w") {
                if (param.as_int() > 1 && param.as_int() < N_p) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_w = param.as_int();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 1-Np");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Q") {
                if (param.as_double_array().size() == 5) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Q_ = param.as_double_array();
                    Q_ = param.as_double_array();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Q must have 5 elements");
                    result.successful = false;
                    result.reason = "Invalid Q size";
                    return result;
                }
            }
            if (param.get_name() == "R") {
                if (param.as_double_array().size() == 3) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> R_ = param.as_double_array();
                    R_ = param.as_double_array();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, R must have 3 elements");
                    result.successful = false;
                    result.reason = "Invalid R size";
                    return result;
                }
            }
            if (param.get_name() == "Rd") {
                if (param.as_double_array().size() == 3) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Rd_ = param.as_double_array();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Rd must have 3 elements");
                    result.successful = false;
                    result.reason = "Invalid Rd size";
                    return result;
                }
            }
            if (param.get_name() == "Dw") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 100) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    D_w_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'D_w'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "r_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 0.5) {
                    r_ref_max_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'r_ref_max'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "r_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() <= 0.0 && param.as_double() > -0.5) {
                    r_ref_min_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'r_ref_min'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_u_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() <= 0.0 && param.as_double() > -0.5) {
                    Delta_u_ref_min_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_u_ref_min'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_u_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 0.5) {
                    Delta_u_ref_max_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_u_ref_max'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_u_tar_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() <= 0.0 && param.as_double() > -0.5) {
                    Delta_u_tar_min_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_u_tar_min'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_u_tar_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 0.5) {
                    Delta_u_tar_max_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_u_tar_max'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_r_ref_min") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() <= 0.0 && param.as_double() > -0.5) {
                    Delta_r_ref_min_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_r_ref_min'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_r_ref_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 0.5) {
                    Delta_r_ref_max_ = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "Invalid value for 'Delta_r_ref_max'");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "path_d"){
                if(param.as_int() >= 0 and param.as_int() <= 1){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    path_d = param.as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-1");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "rho") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 10.0) {
                    rho = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "could not change param value, should be between 0.0-10.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "theta") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= -3.141592 && param.as_double() <= 3.141592) {
                    theta = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_WARN(this->get_logger(), "could not change param value, should be between -3.141592- 3.141592");
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

     /*-----------------------------------------------MPC Funtions---------------------------------------------------*/

    void Precompile(){
        RCLCPP_INFO(this->get_logger(), "Initializing Acados Pf solver with N = %i and Ts = %f", N_p, Ts);
        acados_solver_->init(N_p, Ts);
        p_values_map["Xv_bar"] = Xv_bar;
        p_values_map["Eps_"] = std::vector{Eps_};
        p_values_map["rho_"] = std::vector{rho};
        p_values_map["theta_"] = std::vector{theta};
        switch(path_d) {
            case 0:
                p_values_map["coef"] = std::vector{0.0, 0.0, 1.0, 0.0};  //  a y b Circulo c y d Lineal
                break;
            case 1:
                p_values_map["coef"] = std::vector{30.0, 30.0, 0.0, 0.0};  //  a y b Circulo c y d Lineal
                break;
        }

        if (0 != acados_solver_->set_runtime_parameters(p_values_map)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set NMPC runtime parameters!");
        }

        acados::IndexVector idxbu = {0,1,2};
        acados::ValueVector lbu = {Delta_u_ref_min_/Ts, Delta_u_tar_min_/Ts, Delta_r_ref_min_/Ts};
        acados::ValueVector ubu = {Delta_u_ref_max_/Ts, Delta_u_tar_max_/Ts, Delta_r_ref_max_/Ts};

        acados::IndexVector idxbx = {0,1,3,4,5,6,7};
        acados::ValueVector lbx = {x_e_bar_min_, y_e_bar_min_,  0,  v_bar_min_, u_ref_min_, u_tar_min_, r_ref_min_};
        acados::ValueVector ubx = {x_e_bar_max_, y_e_bar_max_, 10000, v_bar_max_, u_ref_max_, u_tar_max_, r_ref_max_};

        Eigen::MatrixXd W = Eigen::MatrixXd::Zero(Q_.size() + R_.size() + Rd_.size(),
                                                  Q_.size() + R_.size()+ Rd_.size());
        W.diagonal().head(Q_.size()) = Eigen::Map<const Eigen::VectorXd>(Q_.data(), Q_.size());
        W.diagonal().segment(Q_.size(), R_.size()) = Eigen::Map<const Eigen::VectorXd>(R_.data(),  R_.size());
        W.diagonal().tail(Rd_.size()) = Eigen::Map<const Eigen::VectorXd>(Rd_.data(), Rd_.size());

        Eigen::MatrixXd We = Eigen::MatrixXd::Zero(Q_.size() + R_.size(),
                                                   Q_.size() + R_.size());
        We.diagonal().head(Q_.size()) = Eigen::Map<const Eigen::VectorXd>(Q_.data(), Q_.size());
        We.diagonal().tail(R_.size()) = Eigen::Map<const Eigen::VectorXd>(R_.data(), R_.size());

        w_avg.clear();
        w_avg.reserve(N_w);
        w_avg.emplace_back(w_i);

        u_hist.clear();
        u_hist.shrink_to_fit();
        acados::ValueMap u_def;
        u_def["d_u_ref"] = std::vector{0.0};
        u_def["d_u_tar"] = std::vector{0.0};
        u_def["d_r_ref"] = std::vector{0.0};

        con_hist.clear();
        con_hist.shrink_to_fit();

        acados::ValueMap con_map;
        con_map["u_ref"] = std::vector{0.5};
        con_map["u_tar"] = std::vector{0.8};
        con_map["r_ref"] = std::vector{0.0};

        for (int idx = 0; idx <= N_p; idx++) {
            if (idx < N_w){
                float w_k = w_avg.back();
                evolve_w(w_k, 0.4);
                w_avg.emplace_back(w_k);
                W(3, 3) = D_w_;
            }else{
                W(3, 3) = 0.0;
            }
            if (idx>0){
                acados_solver_->set_state_bounds(idx, idxbx, lbx, ubx);
            }
            if (idx==N_p){
                acados::utils::set_cost_W(*acados_solver_, N_p, We);
            }   
            if (idx < N_p){
                acados_solver_->set_control_bounds(idx, idxbu, lbu, ubu);
                acados::utils::set_cost_W(*acados_solver_, idx, W);
                u_hist.emplace_back(u_def);  
                con_hist.emplace_back(con_map);  
            } 
        }
    }

    std::vector<float> spatial_path(const float& w) {
        float x_p, y_p, dx, dy, ddx, ddy;
        switch(path_d) {
            case 0:
                x_p  = w+10;  // 
                y_p  = 10;    // 
                dx   = 1;     // 
                dy   = 0;     //
                ddx   = 0;     // 
                ddy   = 0;     //
                break;
            case 1:
                x_p  = 30-30*cos(w);  
                y_p  = 30*sin(w);     
                dx   = 30*sin(w);     
                dy   = 30*cos(w);   
                ddx   = 30*cos(w);     
                ddy   = -30*sin(w);    
                break;
        }
        float phi  = atan2(dy, dx);
        float dphi = (ddy*dx - ddx*dy)/(dx*dx + dy*dy);
        // Retornar un vector con las variables MX ordenadas
        return {x_p, y_p, phi, dx, dy, dphi};
    }


    bool armed = false, is_first_itr_= true;
    float psi_hat = 0, r_hat = 0, v_hat = 0, x_hat = 0, y_hat = 0, u_d = 0.8, psi_ant;
    float w_i = 0.0;

    float u_ref_ant = 0.5, u_tar_ant = 0.5, r_ref_ant = 0.0;
    int count=0, count_faild = 0, count_ws = 0;
    //------Params-------//
    float Ts;  

    int N_p, N_t, N_w;
    double x_e_bar_max_, x_e_bar_min_, y_e_bar_max_, y_e_bar_min_, v_bar_max_, v_bar_min_, u_ref_max_, u_ref_min_, u_tar_max_, u_tar_min_;
    double r_ref_max_, r_ref_min_, Delta_u_ref_min_, Delta_u_ref_max_, Delta_u_tar_min_,  Delta_u_tar_max_, Delta_r_ref_min_, Delta_r_ref_max_;
    double Eps_;
    double rho, theta;

    int path_d; /*Variable para elegir path*/

    std::vector<double> Xv_bar;
    std::vector<double> Q_, R_, Rd_;
    double D_w_;
    std::vector<float> w_avg;

    std::vector<acados::ValueMap> u_hist;
    std::vector<acados::ValueMap> con_hist;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::ReferenceLlc>::SharedPtr publisher_llc;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_error;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_mpc_state;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_w_pre;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscriber_vel_;

    std::vector<rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr> subs_;

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

};