#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                // Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            // Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        // Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    // Interface state observer
#include <iostream>

#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <gurobi_c++.h>

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

//------------------------------------------------------------------------------------//
//                        DECLARACIÓN DE PARÁMETROS GLOBALES                          //
//------------------------------------------------------------------------------------//

// Declaración de Limites MPC 
const float u_max = 2.0;
const float u_min = 0.01;
const float psi_max = 2 * M_PI;
const float psi_min = 0.0;
const float delta_d_max = 0.4;
const float delta_d_min = -0.4;
const float d_max = 1.0;
const float d_min = -1.0;
const float delta_dz_up = 0.0775;
const float delta_dz_lo = -0.0925;

// Declaración dimención matrices MPC 
const int nx = 3;
const int ny = 2;
const int nu = 2;

// Estructura para los parámetros del modelo
struct Model_Parameters {
    double Xu6;
    double Xu7;
    double Xr10;
    double Xr11;
    double Xr12;
    double Xr13;
};
//------------------------------------------------------------------------------------//

class MpcLlcNode : public rclcpp::Node
{
public:
    MpcLlcNode() : Node("mpc_llc")
    {
        //--------- Parámetros del LLC MPC-------------------//
        this->declare_parameter("Ts", 100.0);
        this->declare_parameter("W_psi", 3.0);
        this->declare_parameter("W_u", 5.0);
        this->declare_parameter("W_dL", 7.0);
        this->declare_parameter("W_dR", 1.0);
        this->declare_parameter("NC", 3);
        this->declare_parameter("NP", 3);

        //----------- Parámetros Modelo --------------------------//
        this->declare_parameter("Xu6", -0.003148);
        this->declare_parameter("Xu7", 0.0810014);
        this->declare_parameter("Xr10", -0.0129643);
        this->declare_parameter("Xr11", -0.0110386);
        this->declare_parameter("Xr12", 0.0);
        this->declare_parameter("Xr13", 0.1717909);

        //--------- Obtener parámetros -------------------//
        Ts = this->get_parameter("Ts").as_double() / 1000.0;
        W_psi = this->get_parameter("W_psi").as_double();
        W_u = this->get_parameter("W_u").as_double();
        W_dL = this->get_parameter("W_dL").as_double();
        W_dR = this->get_parameter("W_dR").as_double();
        NC = this->get_parameter("NC").as_int();
        NP = this->get_parameter("NP").as_int();

        // Inicialización de la estructura de parámetros del modelo
        Parameters_.Xu6 = this->get_parameter("Xu6").as_double();
        Parameters_.Xu7 = this->get_parameter("Xu7").as_double();
        Parameters_.Xr10 = this->get_parameter("Xr10").as_double();
        Parameters_.Xr11 = this->get_parameter("Xr11").as_double();
        Parameters_.Xr12 = this->get_parameter("Xr12").as_double();
        Parameters_.Xr13 = this->get_parameter("Xr13").as_double();

        //----------- Crear Matrices MPC ------------------------//
        Q.resize(NP * ny, NP * ny);
        R.resize(NC * nu, NC * nu);
        Y_ref.resize(NP * ny);

        Q.setZero();
        R.setZero();
        Y_ref.setZero();

        fillSquareMatrix(Q, NP, ny, W_psi, W_u);
        fillSquareMatrix(R, NC, nu, W_dL, W_dR);

        // Print the resulting matrix o vector
        //std::cout << "A matrix:\n" << Umin << std::endl;

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group = cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcLlcNode::param_callback, this, _1));

        // Suscripciones
        subscriber_states_obs_ = this->create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer", rclcpp::SensorDataQoS(), std::bind(&MpcLlcNode::callbackStates,
                this, std::placeholders::_1), options_sensors_);

        subscriber_references_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/control/reference_llc", 1, std::bind(&MpcLlcNode::callbackVelReference,
                this, std::placeholders::_1), options_sensors_);

        subscriber_state = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 1,
            std::bind(&MpcLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this->create_publisher<asv_interfaces::msg::PwmValues>("/control/pwm_value_mpc",
            10);
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcLlcNode::calculateLowLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller MPC Node has been started.");

    }


private:
    void calculateLowLevelController()
    {

        if (armed == false) {
            Y_ref.setZero();
            count = 0;
            dL = 0.0;
            dR = 0.0;
            {
                std::lock_guard<std::mutex> lock(mutex_);

                u_hat = 0.0;
                r_hat = 0.0;
                psi_hat = 0.0;
                sig_u = 0.0;
                sig_r = 0.0;
            }
        }
        else {
            if (count > 5) {
                //auto start = std::chrono::high_resolution_clock::now();
                //RCLCPP_INFO(this->get_logger(), "Ejecutando");
                auto msg = asv_interfaces::msg::PwmValues();

                float u_hat_i;
                float r_hat_i;
                float psi_hat_i;
                float sig_u_i;
                float sig_r_i;
                VectorXd Y_ref_i;
                Y_ref_i.setZero();

                {
                    std::lock_guard<std::mutex> lock(mutex_);

                    Y_ref_i = Y_ref;
                    u_hat_i = u_hat;
                    r_hat_i = r_hat;
                    psi_hat_i = psi_hat;
                    sig_u_i = sig_u;
                    sig_r_i = sig_r;
                }

                //------------------------------------------------------------------------------------//
                //                           CÓDIGO MPC LOW LEVEL                                     //
                //------------------------------------------------------------------------------------//
               
                try {
                    // Inicialización de Gurobi 
                    GRBEnv env = GRBEnv(true);  
                    env.set("LogFile", "mpc.log");  
                    env.start();  
                    GRBModel model = GRBModel(env); 

                    // Definir las Variables de optimización como vectores columna 
                    std::vector<GRBVar> x((NP + 1) * nx); // vector de los estados del sistema a lo largo del horizonte de predicción
                    std::vector<GRBVar> u(NC * nu);       // vector de las entradas del sistema a lo largo del horizonte de control

                    // Definir variables de estado inicial
                    x[0] = model.addVar(psi_hat_i, psi_hat_i, 0.0, GRB_CONTINUOUS, "psi_estado_inicial");
                    x[1] = model.addVar(u_hat_i, u_hat_i, 0.0, GRB_CONTINUOUS, "u_estado_inicial");
                    x[2] = model.addVar(r_hat_i, r_hat_i, 0.0, GRB_CONTINUOUS, "r_estado_inicial");

                    // Definir variables de estado a lo largo del horizonte de predicción
                    for (int k = 1; k <= NP; ++k) {
                        x[k * nx + 0] = model.addVar(psi_min, psi_max, 0.0, GRB_CONTINUOUS, "psi_" + std::to_string(k));
                        x[k * nx + 1] = model.addVar(u_min, u_max, 0.0, GRB_CONTINUOUS, "u_" + std::to_string(k));
                        x[k * nx + 2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "r_" + std::to_string(k));  // Sin límites para r
                    }

                    // Definir variables de control a lo largo del horizonte de control
                    for (int k = 0; k < NC; ++k) {
                        u[k * nu + 0] = model.addVar(d_min, d_max, 0.0, GRB_CONTINUOUS, "dL_" + std::to_string(k));
                        u[k * nu + 1] = model.addVar(d_min, d_max, 0.0, GRB_CONTINUOUS, "dR_" + std::to_string(k));
                    }

                    // -----------------------------------------------------------------------------------------//
                    //                                    FUNCIÓN OBJETIVO                                      //
                    // -----------------------------------------------------------------------------------------//

                    GRBQuadExpr objective = 0;

                    // Penalización del Error de Seguimiento
                    // (y(k+1) - yref(k+1))'*Q*(y(k+1) - yref(k+1))

                    for (int k = 0; k < NP; ++k) {
                        objective += (x[(k + 1) * nx + 0] - Y_ref_i[k * ny + 0]) * Q(k * ny + 0, k * ny + 0) * (x[(k + 1) * nx + 0] - Y_ref_i[k * ny + 0]);
                        objective += (x[(k + 1) * nx + 1] - Y_ref_i[k * ny + 1]) * Q(k * ny + 1, k * ny + 1) * (x[(k + 1) * nx + 1] - Y_ref_i[k * ny + 1]);
                    }

                    // Penalizaión de las acciones de control
                    // (u(k)'*R*u(k)

                    for (int k = 0; k < NC; ++k) {
                        objective += u[k * nu + 0] * R(k * nu + 0, k * nu + 0) * u[k * nu + 0];
                        objective += u[k * nu + 1] * R(k * nu + 1, k * nu + 1) * u[k * nu + 1];
                    }

                    model.setObjective(objective, GRB_MINIMIZE);

                    // -----------------------------------------------------------------------------------------//
                    //                                  RESTRICCIONES  
                    // -----------------------------------------------------------------------------------------// 

                    // Restricciones salidas a lo largo del horizonte de predicción 

                    for (int k = 0; k < NP; ++k) {
                        std::vector<GRBQuadExpr> x_next(nx);
                        Modelo(model, x_next, { x[k * nx], x[k * nx + 1], x[k * nx + 2] }, { u[k * nu], u[k * nu + 1] }, sig_u_i, sig_r_i);

                        // ----------------- Restricciones de actualización de estados --------------------------
                        // Con este bucle estamos imponiendo que x(k+1) se igual a x_next (es decir los estados siguientes calculados con la función modelo).
                     
                        for (int i = 0; i < nx; ++i) {
                            model.addQConstr(x[(k + 1) * nx + i] == x_next[i]);
                        }
                    }

                    // -----------------------------------------------------------------------------------------// 

                     // Restricciones entradas a lo largo del horizonte de control
                     

                    for (int k = 0; k < NC; ++k) {

                        // ------------------ Restricciones de incremento de las entradas ---------------------------

                        if (k == 0) {
                            model.addConstr(u[0] - dL >= delta_d_min);
                            model.addConstr(u[0] - dL <= delta_d_max);
                            model.addConstr(u[1] - dR >= delta_d_min);
                            model.addConstr(u[1] - dR <= delta_d_max);
                        }
                        else {
                            model.addConstr(u[k * nu] - u[(k - 1) * nu] >= delta_d_min);
                            model.addConstr(u[k * nu] - u[(k - 1) * nu] <= delta_d_max);
                            model.addConstr(u[k * nu + 1] - u[(k - 1) * nu + 1] >= delta_d_min);
                            model.addConstr(u[k * nu + 1] - u[(k - 1) * nu + 1] <= delta_d_max);
                        }

                        // ---- Restricción  d_min<=delta<=delta_dz_lo OR delta_dz_up<=delta<=d_max -----------

                        // Definir variables binarias para las restricciones OR

                        GRBVar zL1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                        GRBVar zL2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                        GRBVar zR1 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
                        GRBVar zR2 = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

                        // Restricciones indicadoras para dL
                        model.addGenConstrIndicator(zL1, 1, u[k * nu] >= d_min);
                        model.addGenConstrIndicator(zL1, 1, u[k * nu] <= delta_dz_lo);
                        model.addGenConstrIndicator(zL2, 1, u[k * nu] >= delta_dz_up);
                        model.addGenConstrIndicator(zL2, 1, u[k * nu] <= d_max);

                        // Restricciones indicadoras para dR
                        model.addGenConstrIndicator(zR1, 1, u[k * nu + 1] >= d_min);
                        model.addGenConstrIndicator(zR1, 1, u[k * nu + 1] <= delta_dz_lo);
                        model.addGenConstrIndicator(zR2, 1, u[k * nu + 1] >= delta_dz_up);
                        model.addGenConstrIndicator(zR2, 1, u[k * nu + 1] <= d_max);

                        // Asegurar que al menos una de las restricciones se cumple
                        model.addConstr(zL1 + zL2 == 1); // Sólo una de las dos puede ser verdadera
                        model.addConstr(zR1 + zR2 == 1); // Sólo una de las dos puede ser verdadera
                    }

                    // -----------------------------------------------------------------------------------------//
                    //        OPTIMIZACIÓN DEL MODELO Y APLICACIÓN DE LAS MEJORES ACCIONES DE CONTROL           //
                    // -----------------------------------------------------------------------------------------// 


                    // Optimizar el modelo
                    model.optimize();

                    // Extraer las soluciones y aplicarlas al sistema
                    VectorXd optimal_u1 = VectorXd::Zero(NC);
                    VectorXd optimal_u2 = VectorXd::Zero(NC);

                    for (int k = 0; k < NC; ++k) {
                        optimal_u1[k] = u[k * nu + 0].get(GRB_DoubleAttr_X);
                        optimal_u2[k] = u[k * nu + 1].get(GRB_DoubleAttr_X);
                    }

                    // Asignar valores PWM
                    dL = optimal_u1[0];
                    dR = optimal_u2[0];

                    // -----------------------------------------------------------------------------------------// 

                    // Publicar valores PWM
                    msg.t_left = 400 * dL + 1500;
                    msg.t_righ = 400 * dR + 1500;
                    if (msg.t_left < 1500) {
                        msg.t_left = 1500;
                    }
                    else if (msg.t_left > 1900) {
                        msg.t_left = 1900;
                    }
                    if (msg.t_righ < 1500) {
                        msg.t_righ = 1500;
                    }
                    else if (msg.t_righ > 1900) {
                        msg.t_righ = 1900;
                    }

                    publisher_pwm->publish(msg);

                    // auto end = std::chrono::high_resolution_clock::now();
                    // std::chrono::duration<double> elapsed = end - start;
                    // double miliseconds = elapsed.count()*1000;
                    // Imprime el tiempo con dos decimales fijos
                    // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);

                }
                // -------------------- Manejo de excepciones con Gurobi ---------------------------------- // 
                catch (GRBException& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error code = %d, %s", e.getErrorCode(), e.getMessage().c_str());
                    // Publicar valores PWM
                    msg.t_left = 1500;
                    msg.t_righ = 1500;
                    publisher_pwm->publish(msg);
                }
                catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Exception during optimization");
                    msg.t_left = 1500;
                    msg.t_righ = 1500;
                    publisher_pwm->publish(msg);
                }
                // -----------------------------------------------------------------------------------------// 
            }
            else {
                auto msg = asv_interfaces::msg::PwmValues();
                msg.t_left = 1500;
                msg.t_righ = 1500;
                count++;
                publisher_pwm->publish(msg);
            }
        }
    }

   //--------------------------------------------------------------------------------------------------------------------//
   //                                              FUNCIÓN MODELO
   //--------------------------------------------------------------------------------------------------------------------//

    void Modelo(GRBModel& model, std::vector<GRBQuadExpr>& x_next, const std::vector<GRBVar>& x, const std::vector<GRBVar>& u,const double sigma_u_i, const double sigma_r_i) {
        // Descomposición del vector de estados
        GRBVar Psi = x[0];
        GRBVar u_var = x[1];
        GRBVar r_var = x[2];

        // Descomposición del vector de entradas
        GRBVar Delta_L = u[0];
        GRBVar Delta_R = u[1];

        // Crear la variable binaria Beta
        GRBVar Beta = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "Beta");

        // Si Delta_L >= 0 y Delta_R >= 0, entonces Beta = 1
        model.addGenConstrIndicator(Beta, 1, Delta_L, GRB_GREATER_EQUAL, 0);
        model.addGenConstrIndicator(Beta, 1, Delta_R, GRB_GREATER_EQUAL, 0);

        // Si Delta_L < 0 o Delta_R < 0, entonces Beta = 0
        model.addGenConstrIndicator(Beta, 0, Delta_L, GRB_LESS_EQUAL, -1e-6);
        model.addGenConstrIndicator(Beta, 0, Delta_R, GRB_LESS_EQUAL, -1e-6);

        // Variables auxiliares para términos cuadrados y cúbicos
        GRBVar Delta_mean = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Delta_mean");
        GRBVar Delta_mean_square = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Delta_mean_square");
        GRBVar Delta_diff = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Delta_diff");
        GRBVar Delta_diff_square = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Delta_diff_square");
    
        // Restricciones para las variables auxiliares
        model.addConstr(Delta_mean == (Delta_R + Delta_L)/2, "Delta_mean_constr");
        model.addQConstr(Delta_mean_square == Delta_mean * Delta_mean, "Delta_mean_square_constr");
        model.addConstr(Delta_diff == Delta_R - Delta_L, "Delta_diff_constr");
        model.addQConstr(Delta_diff_square == Delta_diff * Delta_diff, "Delta_diff_square_constr");

        // Crear variable Alpha que varía entre -1 y 1
        GRBVar Alpha = model.addVar(-1.0, 1.0, 0.0, GRB_CONTINUOUS, "Alpha");       
        GRBVar Alpha_aux = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "Alpha_aux");// Crear la variable binaria auxiliar Alpha_aux
        model.addConstr(Alpha == 2 * Alpha_aux - 1, "Definir_Alpha"); // Definir Alpha en términos de Alpha_aux     
        model.addGenConstrIndicator(Alpha_aux, 1, Delta_diff, GRB_GREATER_EQUAL, 0);// Si Delta_diff >= 0 Alpha_aux = 1 (Alpha = 1)
        model.addGenConstrIndicator(Alpha_aux, 0, Delta_diff, GRB_LESS_EQUAL, -1e-6); // Si Delta_diff < 0 Alpha_aux = 0 (Alpha = -1)
        GRBVar alfaxbeta = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Alfa_beta");
        model.addQConstr(alfaxbeta == (1-Beta) * Alpha , "Alfa_beta_constr");

        // Aplicación de las ecuaciones del modelo para calcular los siguientes estados
        x_next[0] = Psi + Ts * r_var;

        x_next[1] = u_var + Ts * ( Parameters_.Xu6 * (Delta_mean_square + Delta_diff_square/4) +
                                Parameters_.Xu7 * Delta_mean + sigma_u_i);
    
        x_next[2] = r_var + Ts * ( Parameters_.Xr10 * alfaxbeta * (Delta_mean_square + Delta_diff_square/4) +
                                Parameters_.Xr11 * Delta_mean * Delta_diff +
                                Parameters_.Xr12 * alfaxbeta * Delta_mean +
                                Parameters_.Xr13 * Delta_diff/2 + sigma_r_i);
    }

    //--------------------------------------------------------------------------------------------------------------------//
    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_hat = msg->velocity.x;
            r_hat = msg->velocity.z;
            psi_hat = msg->point.z;
            sig_u = msg->disturbances.x;
            sig_r = msg->disturbances.z;
        }
    }

    //  u_ref = msg->x;
    //  r_ref = msg->y;
    //  psi_ref = msg->z;

    void callbackVelReference(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // Fill the y_ref vector
            for (int i = 0; i < ny * NP; i += ny) {
                Y_ref.segment(i, ny) << msg->z, msg->x;
            }
        }
    }

    // Función para calcular la salida del filtro de derivación
    float derivationFilter(float input, std::vector<float>& memory, float a, float b) {
        float output = (a * memory[0]) + b * (input - memory[1]);
        // Actualizar memoria para la próxima iteración
        memory[0] = output;
        memory[1] = input;
        return output;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed = msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    void fillSquareMatrix(MatrixXd& M, int const size, int const sub_size, float const W1, float const W2) {
        // Loop to fill the matrix Q with diagonal blocks
        for (int i = 0; i < size; ++i) {
            // Create the diagonal 
            VectorXd diag_block(sub_size);
            diag_block << W1, W2;      // Adjust based on the actual ny value and the values you want to set

            // Set the diagonal block in the matrix Q       
            M.block(i * sub_size, i * sub_size, sub_size, sub_size) = diag_block.asDiagonal();
        }
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto& param : params) {
            if (param.get_name() == "W_psi") {
                if (param.as_double() >= 0.0 and param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_psi = param.as_double();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_u") {
                if (param.as_double() >= 0.0 and param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_u = param.as_double();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_dL") {
                if (param.as_double() >= 0.0 and param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_dL = param.as_double();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_dR") {
                if (param.as_double() >= 0.0 and param.as_double() < 100.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    W_dR = param.as_double();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "NC") {
                if (param.as_int() >= 1 and param.as_int() < 30) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NC = param.as_int();

                    R.resize(NC * nu, NC * nu);
                    R.setZero();
                    fillSquareMatrix(R, NC, nu, this->W_dL, this->W_dR);

                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "NP") {
                if (param.as_int() >= 1 and param.as_int() < 30) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NP = param.as_int();
                    Y_ref.resize(NP * ny);
                    Q.resize(NP * ny, NP * ny);
                    Q.setZero();
                    Y_ref.setZero();
                    fillSquareMatrix(Q, NP, ny, this->W_psi, this->W_u);

                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
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
    /******/
    bool armed = false;  // PILAS DEBE SER FALSE PARA IMPLEMENTAR
    /******/
    float sig_u, sig_r, u_hat, r_hat, psi_hat;
    int count = 0;

    //------Params-------//
    float Ts;
    float dL, dR;

    // Instancia de la estructura Model_Parameters

    Model_Parameters Parameters_;

    //Parámetros del controlador MPC

    float W_psi, W_u, W_dL, W_dR;
    int NC, NP;

    /* Definicion Matrices y Vectores*/
    MatrixXd Q;
    MatrixXd R;
    VectorXd Y_ref;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr publisher_pwm;
    rclcpp::TimerBase::SharedPtr timer_;

    // mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;

    OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcLlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}