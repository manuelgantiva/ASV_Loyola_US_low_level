#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer


#include <cmath>
#include <thread>
#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>

using namespace std;
using namespace Eigen;

using casadi::MX;
using casadi::Function;
using casadi::DM;
using casadi::abs;
using casadi::MXVector;
using casadi::Slice;


using std::placeholders::_1;

class MpcLlcNode : public rclcpp::Node
{
public:
    MpcLlcNode() : Node("mpc_llc")
    {     
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);

        this->declare_parameter<int>("N_p", 10);
        this->declare_parameter<int>("N_c", 10);
        this->declare_parameter<std::vector<double>>("Q", {1.0, 1.0, 1.0, 1.0});
        this->declare_parameter<std::vector<double>>("R", {0.1, 0.1});
        this->declare_parameter<std::vector<double>>("Nu", {1.0, 1.0});

        this->declare_parameter<double>("u_max", 1.5);
        this->declare_parameter<double>("u_min", 0.0);
        this->declare_parameter<double>("v_max", 0.5);
        this->declare_parameter<double>("v_min", -0.5);
        this->declare_parameter<double>("r_max", 0.8);
        this->declare_parameter<double>("r_min", -0.8);
        this->declare_parameter<double>("Delta_d_max", 0.2);
        this->declare_parameter<double>("Delta_d_min", -0.2);

        this-> declare_parameter("Xu", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xv", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Xr", std::vector<double>{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
        this-> declare_parameter("Dz_up", 0.0750);
        this-> declare_parameter("Dz_down", -0.08);

        this-> declare_parameter("Sig_on", false);
        this->declare_parameter<double>("max_cpu_time", 0.08);
        

        Ts = this->get_parameter("Ts").as_double()/1000.0;

        my_id = (this->get_parameter("my_id").as_string());

        // Leer parámetros y asignar a variables miembro
        this->get_parameter("N_p", N_p);
        this->get_parameter("N_c", N_c);
        std::vector<double> Q_ = this->get_parameter("Q").as_double_array();
        std::vector<double> R_ = this->get_parameter("R").as_double_array();
        std::vector<double> Nu_ = this->get_parameter("Nu").as_double_array();

        this->get_parameter("u_max", u_max_);
        this->get_parameter("u_min", u_min_);
        this->get_parameter("v_max", v_max_);
        this->get_parameter("v_min", v_min_);
        this->get_parameter("r_max", r_max_);
        this->get_parameter("r_min", r_min_);
        this->get_parameter("Delta_d_max", Delta_d_max_);
        this->get_parameter("Delta_d_min", Delta_d_min_);

        Xu = this->get_parameter("Xu").as_double_array();
        Xv = this->get_parameter("Xv").as_double_array();
        Xr = this->get_parameter("Xr").as_double_array();
        Dz_up  = this->get_parameter("Dz_up").as_double();
        Dz_down = this->get_parameter("Dz_down").as_double();

        RCLCPP_INFO(this->get_logger(), "Low Level Controller MPC Node in %s has been started.", my_id.c_str());
        sig_on = static_cast<float>(this->get_parameter("Sig_on").as_bool());
        this->get_parameter("max_cpu_time", Max_time);

        Q = casadi::DM::zeros(4, 4);
        Q(0, 0) = Q_[0];
        Q(1, 1) = Q_[1];
        Q(2, 2) = Q_[2];
        Q(3, 3) = Q_[3];

        R = casadi::DM::zeros(2, 2);
        R(0, 0) = R_[0];
        R(1, 1) = R_[1];

        Nu = casadi::DM::zeros(2, 2);
        Nu(0, 0) = Nu_[0];
        Nu(1, 1) = Nu_[1];

        Precompile();

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcLlcNode::calculateLowLevelController, this), cb_group_obs_);

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcLlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&MpcLlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<geometry_msgs::msg::Vector3>(
            "/" + my_id + "/control/reference_llc", 1, std::bind(&MpcLlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&MpcLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/" + my_id + "/control/pwm_value_mpc",
                1);
        publisher_mps_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/mpc_state",1);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller MPC Node in %s has been started.", my_id.c_str());
    	
    }

private:
    void calculateLowLevelController()
    {
        if(armed==false){
            count=0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                u_hat = 0.0;
                v_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;
                sig_u = 0.0;
                sig_v = 0.0;
                sig_r = 0.0;
                u_ref = 0.2;
                r_ref = 0.0;
                psi_ref = 0.0;
                count_faild = 0;
                U_ant = casadi::DM::zeros(n_controls, 1);
                u_nc_prev = casadi::DM::ones(n_controls, N_c) * 0.3;
            }
        }else{
            //auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::PwmValues();

            if(count > 8){
                double u_hat_i;
                double v_hat_i;
                double r_hat_i;
                double psi_hat_i;
                double sig_u_i;
                double sig_v_i;
                double sig_r_i;

                double u_ref_i;
                double r_ref_i;
                double psi_ref_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    u_hat_i= u_hat;
                    v_hat_i= v_hat;
                    r_hat_i=r_hat;
                    psi_hat_i=psi_hat;
                    sig_u_i=sig_u;
                    sig_v_i=sig_v;
                    sig_r_i=sig_r;
                    u_ref_i= u_ref;
                    r_ref_i= r_ref; //r_ref;
                    psi_ref_i= psi_ref;
                }


                // Estado inicial
                DM x0_init = DM::vertcat({u_hat_i, v_hat_i, r_hat_i, psi_hat_i});  // (4x1) si n_states == 4

                // Paso 2: Estados de referencia a lo largo del horizonte
                DM ref_block = DM::vertcat({
                    DM(u_ref_i),
                    DM(0.0),
                    DM(r_ref_i),
                    DM(psi_ref_i)
                }); // tamaño: (4x1)

                std::vector<DM> ref_blocks;
                for (int i = 0; i < N_p; ++i) {
                    ref_blocks.push_back(ref_block);
                }
                DM ref_vec = DM::vertcat(ref_blocks); // tamaño final: (4 * N_p x 1)
                // MX ref_vec = MX::repmat(ref_block, N_p, 1);

                DM u_prev = U_ant;  // (n_controls x 1)

                // Paso 4: Perturvaciones Externas
                DM se_0 = DM::vertcat({sig_u_i, sig_v_i, sig_r_i});  // (3x1) si n_inputs == 3

                /*std::cout << "x0_init shape: " << x0_init.size1() << " x " << x0_init.size2() << std::endl;
                std::cout << "ref_vec shape: " << ref_vec.size1() << " x " << ref_vec.size2() << std::endl;
                std::cout << "u_prev shape: " << u_prev.size1() << " x " << u_prev.size2() << std::endl;
                std::cout << "se_0 shape: " << se_0.size1() << " x " << se_0.size2() << std::endl;*/
                

                DM p_vector = DM::vertcat({
                    x0_init,         // (n_states x 1)
                    ref_vec,         // (n_states * N_p x 1)
                    DM::zeros(n_controls * N_c, 1), // (n_controls * N_c x 1)
                    u_prev,           // (n_controls x 1)
                    se_0              // (n_inputs x 1)
                });
            
                // Selecciona las filas de la 1 a la N_c-1
                DM left = u_nc_prev(Slice(), Slice(1, N_c));  // columnas 1 hasta N_c-1
                DM last_col = u_nc_prev(Slice(), Slice(N_c - 1, N_c)); // última columna
                DM u_nc_shifted = DM::horzcat({left, last_col});

                DM x0 = DM::reshape(u_nc_shifted, N_c * n_controls, 1); // (N_c * n_controls, 1)

                /*std::stringstream ss_lbg;
                ss_lbg << x0;
                RCLCPP_INFO(this->get_logger(), "x0: %s", ss_lbg.str().c_str());
                std::cout << "Total constraints (u_nc_prev): " << x0.nnz() << std::endl;
                std::cout << "u_nc_prev shape: " << x0.size1() << " x " << u_nc_prev.size2() << std::endl;
                std::cout << "x0 shape: " << x0.size1() << " x " << x0.size2() << std::endl;*/

                // Entrada al solver
                std::map<std::string, casadi::DM> solver_args;
                solver_args["x0"] = x0;
                solver_args["lbx"] = args["lbx"];
                solver_args["ubx"] = args["ubx"];
                solver_args["lbg"] = args["lbg"];
                solver_args["ubg"] = args["ubg"];
                solver_args["p"] = p_vector;
                
                // Ejecutar el solver de forma NUMÉRICA
                std::map<std::string, casadi::DM> sol = solver(solver_args); 

                casadi::Dict stats = solver.stats();
                // bool success = static_cast<bool>(stats["success"]);
                int n_iter = static_cast<int>(stats["iter_count"]);
                // double t_proc = static_cast<double>(stats["t_proc_solver"]); // tiempo de CPU (segundos)
                std::string status = static_cast<std::string>(stats["return_status"]);

                // Procesar resultado
                DM j_min;
                int t_proc;
                if (status == "Solve_Succeeded" || status == "Solved_To_Acceptable_Level") {
                    u_nc_prev = reshape(sol.at("x"), n_controls, N_c);
                    j_min = sol.at("f");
                    t_proc = 0;
                } else if (status == "Maximum_CpuTime_Exceeded" || status == "Maximum_Iterations_Exceeded") {
                    RCLCPP_INFO(this->get_logger(), "Fallo Tiempo");
                    u_nc_prev = reshape(sol.at("x"), n_controls, N_c);
                    j_min = sol.at("f");
                    t_proc = 1;
                } else {
                    DM left = u_nc_prev(Slice(), Slice(1, N_c));  // columnas 1 hasta N_c-1
                    DM last_col = u_nc_prev(Slice(), Slice(N_c - 1, N_c)); // última columna
                    u_nc_prev = DM::horzcat({left, last_col});
                    j_min = -1;
                }

                U_ant = u_nc_prev(Slice(), 0);

                double L = static_cast<double>(U_ant(0,0));
                double R = static_cast<double>(U_ant(1,0));

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

                auto msg_s = geometry_msgs::msg::Vector3();
                msg_s.x = static_cast<double>(j_min);
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
            sig_u = sig_on*msg->disturbances.x;
            sig_v = sig_on*msg->disturbances.y;
            sig_r = sig_on*msg->disturbances.z;
        }
    }

    void callbackVelReference(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_ref = msg->x;
            r_ref = msg->y;
            psi_ref = msg->z;
        }
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "N_p") {
                if (param.as_int() > 1 && param.as_int() < 30) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_p = param.as_int();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 1-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "N_c") {
                if (param.as_int() > 0 && param.as_int() < N_p) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    N_c = param.as_int();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 1-Np");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Q") {
                if (param.as_double_array().size() == 4) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Q_ = param.as_double_array();
                     Q = casadi::DM::zeros(4, 4);
                    Q(0, 0) = Q_[0];
                    Q(1, 1) = Q_[1];
                    Q(2, 2) = Q_[2];
                    Q(3, 3) = Q_[3];
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Q must have 4 elements");
                    result.successful = false;
                    result.reason = "Invalid Q size";
                    return result;
                }
            }
            if (param.get_name() == "R") {
                if (param.as_double_array().size() == 2) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> R_ = param.as_double_array();
                    R = casadi::DM::zeros(2, 2);
                    R(0, 0) = R_[0];
                    R(1, 1) = R_[1];
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, R must have 2 elements");
                    result.successful = false;
                    result.reason = "Invalid R size";
                    return result;
                }
            }
            if (param.get_name() == "Nu") {
                if (param.as_double_array().size() == 2) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> Nu_ = param.as_double_array();
                    Nu = casadi::DM::zeros(2, 2);
                    Nu(0, 0) = Nu_[0];
                    Nu(1, 1) = Nu_[1];
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Nu must have 2 elements");
                    result.successful = false;
                    result.reason = "Invalid Nu size";
                    return result;
                }
            }
            if (param.get_name() == "Delta_d_max") {
                if (param.as_double() >= 0.0 && param.as_double() <= 1.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    // Delta_d_max = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Delta_d_max should be between 0.0 and 1.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_d_min") {
                if (param.as_double() >= -1.0 && param.as_double() <= 0.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    // Delta_d_min = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, Delta_d_min should be between -1.0 and 0.0");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Sig_on") {
                RCLCPP_INFO(this->get_logger(), "changed param value");
                // Sig_on = param.as_bool();
            }
            if (param.get_name() == "max_cpu_time") {
                if (param.as_double() > 0.0 && param.as_double() < Ts) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Max_time = param.as_double();
                    Precompile();
                } else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, max_cpu_time should be between 0.001 and Ts");
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
        Function f_model;
        MX X_LL, U_LL, Param;

        create_dynamic_model(f_model);
        create_predictive_model(f_model, X_LL, U_LL, Param);
        MX obj_fc = build_objective(X_LL, U_LL, Param);
        create_solver(obj_fc, X_LL, U_LL, Param);

        U_ant = casadi::DM::zeros(n_controls, 1);
        u_nc_prev = casadi::DM::ones(n_controls, N_c) * 0.3;
        u_def = u_nc_prev;  //  Opcion optima por defecto 
    }
           
    void create_dynamic_model(Function& f_model) {
        MX u = MX::sym("u");
        MX v = MX::sym("v");
        MX r = MX::sym("r");
        MX psi = MX::sym("psi");

        MX d_l = MX::sym("d_l");
        MX d_r = MX::sym("d_r");

        // Sigmas Externas
        MX se_u = MX::sym("se_u");
        MX se_v = MX::sym("se_v");
        MX se_r = MX::sym("se_r");

        MXVector states = {u, v, r, psi};
        MXVector controls = {d_l, d_r};
        MXVector disturbances = {se_u, se_v, se_r};

        n_states = states.size();
        n_controls = controls.size();
        n_inputs = disturbances.size();

        // Media y diferencia
        MX d_mean = (d_l + d_r) / 2.0;
        MX d_diff = d_l - d_r;

        MX sig = if_else(d_diff >= 0, 1, -1);

        MX cond_l = d_l >= 0;
        MX cond_r = d_r >= 0;

        // beta es 1 si ambos >= 0, 0 si no
        MX beta = MX::if_else(cond_l && cond_r, MX(1.0), MX(0.0));

        MX sum_1 = ((d_mean*d_mean)+((d_diff*d_diff)/4.0));

        MX IG = MX::zeros(n_states);

        IG(0) = (Xu[4]*sum_1)+(Xu[5]*d_mean);
        IG(1) = (Xv[8]*sum_1*(1-beta)*sig)+(Xv[9]*d_mean*d_diff)+(Xv[10]*d_mean*(1-beta)*sig)+(Xv[11]*d_diff/2.0);
        IG(2) = (Xr[8]*sum_1*(1-beta)*sig)+(Xr[9]*d_mean*d_diff)+(Xr[10]*d_mean*(1-beta)*sig)+(Xr[11]*d_diff/2.0);
        IG(3) = r;

        // Matriz D
        MX sigma = MX::zeros(n_states);
        sigma(0) = (Xu[0] * u * abs(u) + Xu[1] * v * r + Xu[2] * r*r + Xu[3] * u) + se_u;
        sigma(1) = (Xv[0] * v * abs(v) + Xv[1] * v * abs(r) + Xv[2] * r * abs(v) + Xv[3] * r * abs(r) + Xv[4] * u * v 
                     + Xv[5] * u * r + Xv[6] * v + Xv[7] * r) + se_v;
        sigma(2) = (Xr[0] * v * abs(v) + Xr[1] * v * abs(r) + Xr[2] * r * abs(v) + Xr[3] * r * abs(r) + Xr[4] * u * v 
                     + Xr[5] * u * r + Xr[6] * v + Xr[7] * r) + se_r;
        sigma(3) = 0;

        MX model = (IG + sigma);

        f_model = Function("f", {MX::vertcat(states), MX::vertcat(controls), MX::vertcat(disturbances)}, {model});
    }

    void create_predictive_model(Function& f_model, MX& X_LL, MX& U_LL, MX& P_LL) {
        // 1. Variables simbólicas para controles y estados
        U_LL = MX::sym("U_LL", n_controls, N_c);
        X_LL = MX::sym("X_LL", n_states, N_p + 1);

        // 2. Vector de parámetros
        // - estados iniciales 4
        // - referencias de estado (N_p veces) 4*10 = 40
        // - referencias de control (N_c veces) 2*5 = 10
        // - último control aplicado 2
        // - Perturbaciones Externas observadas en el instante actual 3
        int total_param_size = n_states + N_p * n_states + N_c * n_controls + n_controls + n_inputs;
        P_LL = MX::sym("P_LL", total_param_size);

        // 3. Asignar el estado inicial
        MXVector x_cols(N_p + 1);  // columnas de X_LL

        x_cols[0] = P_LL(casadi::Slice(0, n_states));  // estado inicial


        int idx_inputs = total_param_size - n_inputs;
        MX se = P_LL(casadi::Slice(idx_inputs, total_param_size));       

        for (int k = 0; k < N_p; ++k)
        {
            MX st = x_cols[k];

            MX con;
            if (k < N_c)
                con = U_LL(casadi::Slice(), k);
            else
                con = U_LL(casadi::Slice(), N_c - 1);  // mantiene el último control

            MX f_val = f_model(MXVector{st, con, se})[0];  // f_model: f(x,u)
            MX st_next = st + Ts * f_val;

            x_cols[k + 1] = st_next;
        }

        // 4. Reconstruir la matriz X_LL con las columnas computadas
        X_LL = horzcat(x_cols);
    }

    MX build_objective(const MX& X_LL, const MX& U_LL, const MX& P_LL){
        MX Q_sx = MX(Q);
        MX R_sx = MX(R);
        MX Nu_sx = MX(Nu);
        MX obj_LL = MX(0);

        for (int k = 1; k <= N_p; ++k)
        {
            MX st = X_LL(Slice(), k);  // Estado en k
            MX ref_st = P_LL(Slice(n_states * k, n_states * (k + 1)));  // Referencia de estado
            MX diff_st = st - ref_st;
            MX cost_st = casadi::MX::mtimes(casadi::MX::mtimes(diff_st.T(), Q_sx), diff_st);  // (1x4)*(4x4)*(4x1)
            MX cost_con = MX(0);
            MX cost_ref_con = MX(0);

            if (k <= N_c)
            {
                MX con = U_LL(Slice(), k - 1);  // Control en k-1
                cost_con = casadi::MX::mtimes(casadi::MX::mtimes(con.T(), R_sx), con);  // (1x2)*(2x2)*(2x1)

                int idx_ref = n_states + n_states * N_p + n_controls * (k - 1);
                MX ref_con = P_LL(Slice(idx_ref, idx_ref + n_controls));

                MX diff_con = con - ref_con;
                cost_ref_con = casadi::MX::mtimes(casadi::MX::mtimes(diff_con.T(), Nu_sx), diff_con);  // (1x2)*(2x2)*(2x1)
            }
            
            obj_LL += cost_st + cost_con + cost_ref_con;
        }

        return obj_LL;
    }

    void create_solver(const casadi::MX& obj_fc, const casadi::MX& X_LL,
                        const casadi::MX& U_LL, const casadi::MX& P_LL){
        MX g_LL; 

        // Restricciones de estados
        for (int k = 1; k <= N_p; ++k) {
            g_LL = MX::vertcat({g_LL, X_LL(Slice(), k)});
        }

        int ng = g_LL.size1();

        DM lbg = DM::zeros(ng,1);
        DM ubg = DM::zeros(ng,1);

        for (int k = 0; k < N_p; ++k) {
            int base = k * n_states;
            lbg(base + 0) = u_min_;
            lbg(base + 1) = v_min_;
            lbg(base + 2) = r_min_;
            lbg(base + 3) = -casadi::inf;;

            ubg(base + 0) = u_max_;
            ubg(base + 1) = v_max_;
            ubg(base + 2) = r_max_;
            ubg(base + 3) = casadi::inf;
        }

        // Restricciones: delta u respecto a control anterior (desde P_LL)
        for (int i = 0; i < n_controls; ++i) {
            int idx = n_states + N_p * n_states + N_c * n_controls + i;
            MX u0 = U_LL(i, 0);
            MX prev = P_LL(idx);
            g_LL = MX::vertcat({g_LL, u0 - prev});
        }

        // Delta_u entre pasos k y k-1
        for (int k = 1; k < N_c; ++k) {
            for (int i = 0; i < n_controls; ++i) {
                MX delta_u = U_LL(i, k) - U_LL(i, k - 1);
                g_LL = MX::vertcat({g_LL, delta_u});
            }
        }

        int n_delta = N_c * n_controls;

        DM lbg_delta = DM::zeros(n_delta,1);
        DM ubg_delta = DM::zeros(n_delta,1);
        for (int k = 0; k < n_delta; k += 2) {
            lbg_delta(k + 0) = Delta_d_min_;
            lbg_delta(k + 1) = Delta_d_min_;
            ubg_delta(k + 0) = Delta_d_max_;
            ubg_delta(k + 1) = Delta_d_max_;
        }

        lbg = DM::vertcat({lbg, lbg_delta});
        ubg = DM::vertcat({ubg, ubg_delta});

         // Saturación: reverse- reverser
        MX sat_LL;
        for (int k = 0; k < N_c; ++k) {
            MX d_l = U_LL(0, k);
            MX d_r = U_LL(1, k);
            MX avg = (d_l + d_r)/2;
            MX dif = (d_l - d_r);

            sat_LL = MX::vertcat({
                sat_LL,
                avg + abs(dif) / 2});
        }

        int sat_size =  N_c;
        DM lbg_sat = DM::zeros(sat_size,1);
        DM ubg_sat = DM::zeros(sat_size,1);

        for (int k = 0; k < N_c; ++k) {
            lbg_sat(k) = 0.0;
            ubg_sat(k) = casadi::inf;
        }

        g_LL = MX::vertcat({g_LL, sat_LL});
        lbg = DM::vertcat({lbg, lbg_sat});
        ubg = DM::vertcat({ubg, ubg_sat});


        DM lbx = DM::zeros(n_controls * N_c, 1);
        DM ubx = DM::zeros(n_controls * N_c, 1);

        for (int k = 0; k < N_c; ++k) {
            int idx_dl = k * n_controls + 0;
            int idx_dr = k * n_controls + 1;

            lbx(idx_dl) =  -1.0 - Dz_down;
            ubx(idx_dl) =  1.0 - Dz_up;

            lbx(idx_dr) = -1.0 - Dz_down;
            ubx(idx_dr) =  1.0- Dz_up;
        }

        // Variables de decisión
        casadi::MX OPT_variables = casadi::MX::reshape(U_LL, n_controls * N_c, 1);

        // Definir el problema de optimización
        std::map<std::string, casadi::MX> nlp_prob;
        nlp_prob["f"] = obj_fc;         // Costo
        nlp_prob["x"] = OPT_variables;  // Variables de decisión
        nlp_prob["g"] = g_LL;           // Restricciones
        nlp_prob["p"] = P_LL;           // Parámetros externos


        casadi::Dict opts;
        opts["ipopt.max_iter"] = 1000;
        opts["ipopt.print_level"] = 5;               // Nivel de detalle (1-12, pon 5 o más para ver todo)
        opts["ipopt.sb"] = "yes";                    // Silence banner, si quieres
        opts["ipopt.output_file"] = "ipopt_log.txt"; // Aquí se guardará todo el output
        opts["ipopt.acceptable_tol"] = 1e-3;
        opts["ipopt.acceptable_obj_change_tol"] = 1e-2;
        opts["ipopt.max_cpu_time"] = Max_time;


        solver = casadi::nlpsol("solver", "ipopt", nlp_prob, opts);

        args["lbg"] = lbg;
        args["ubg"] = ubg;
        args["lbx"] = lbx;
        args["ubx"] = ubx;
    }

    bool armed = false;
    float u_hat = 0.0, v_hat = 0, psi_hat = 0, r_hat = 0, sig_u = 0, sig_v = 0, sig_r = 0, u_ref = 0.2, psi_ref = 0, r_ref = 0;
    int count=0, count_faild = 0;
    //------Params-------//
    float Ts, sig_on, Max_time;  

    int N_p, N_c;
    double u_max_, u_min_, v_max_, v_min_, r_max_, r_min_, Delta_d_max_, Delta_d_min_;
    float Dz_up, Dz_down;  

    std::vector<double> Xu, Xv, Xr;
    casadi::DM Q, R, Nu;

    std::map<std::string, casadi::DM> args;
    Function solver;
    int n_states, n_controls, n_inputs;
    DM U_ant, u_nc_prev, u_def;


    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_mps_state;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_IG;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcLlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}