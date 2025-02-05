#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                // Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            // Interface master reference mpc hlc x->meta y->actual z->vecino
#include "std_msgs/msg/float64.hpp"                 // Interface reference hlc
// #include "asv_library/curvas_sim.h"
#include "asv_library/curvas_alamillo.h"

#include <cmath>
#include <thread>
#include <Eigen/Dense>
#include <gurobi_c++.h>

using namespace std;
using namespace Eigen;
using std::placeholders::_1;

//------------------------------------------------------------------------------------//
//                        DECLARACI�N DE PAR�METROS GLOBALES                          //
//------------------------------------------------------------------------------------//

// Declaraci�n dimenci�n matrices MPC 
const int nx = 3;           // Numero de estados medidos del modelo  3 -> w_v, w_m , w_s
const int ny = 3;           // Numero de salidas del modelo  3 -> w_v, w_m - w_s , w_v-w_s
const int nu = 3;           // Numero de entradas del modelo  3 -> ud_v, ud_m, ud_s

//------------------------------------------------------------------------------------//

class MpcHlcNode : public rclcpp::Node
{
public:
    MpcHlcNode() : Node("mpc_hlc")
    {
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        //--------- Par�metros del HLC MPC-------------------//
        this->declare_parameter("Ts", 5.0);
        this-> declare_parameter("circuit", 0);     // Circuit de tres curvas seleccionado
        this-> declare_parameter("path_max", true); // Curva asignada al maestro
        
        this->declare_parameter("NC", 10);
        this->declare_parameter("NP", 10);

        this->declare_parameter("C_1", 0.1);

        this-> declare_parameter("W_Q", std::vector<float>{1.0, 1.0, 1.0});
        this-> declare_parameter("W_R", std::vector<float>{1.0, 1.0, 1.0});
        this-> declare_parameter("W_P", std::vector<float>{1.0, 1.0, 1.0});
        this-> declare_parameter("W_N", std::vector<float>{1.0, 1.0, 1.0});

        this-> declare_parameter("Ud_min", std::vector<float>{0.2, 0.2, 0.2});
        this-> declare_parameter("Ud_max", std::vector<float>{0.8, 0.8, 0.8});

        this-> declare_parameter("Delta_Ud_min", std::vector<float>{-0.4, -0.4, -0.4});
        this-> declare_parameter("Delta_Ud_max", std::vector<float>{0.4, 0.4, 0.4});
        
        //--------- Obtener par�metros -------------------//
        my_id = (this->get_parameter("my_id").as_string());
        Ts = this->get_parameter("Ts").as_double();
        circuit  = this->get_parameter("circuit").as_int();
        path_max  = this->get_parameter("path_max").as_bool();

        NC = this->get_parameter("NC").as_int();
        NP = this->get_parameter("NP").as_int();

        C_1 = this->get_parameter("C_1").as_double();

        std::vector<double> W_Q = this->get_parameter("W_Q").as_double_array();
        std::vector<double> W_R = this->get_parameter("W_R").as_double_array();
        std::vector<double> W_P = this->get_parameter("W_P").as_double_array();
        std::vector<double> W_N = this->get_parameter("W_N").as_double_array();

        Ud_min = this->get_parameter("Ud_min").as_double_array();
        Ud_max = this->get_parameter("Ud_max").as_double_array();
        Delta_Ud_min = this->get_parameter("Delta_Ud_min").as_double_array();
        Delta_Ud_max = this->get_parameter("Delta_Ud_max").as_double_array();
        
        
        //----------- Crear Matrices MPC ------------------------//
        Q << W_Q[0], 0.0, 0.0,
             0.0, W_Q[1], 0.0,
             0.0, 0.0, W_Q[2];

        R << W_R[0], 0.0, 0.0,
             0.0, W_R[1], 0.0,
             0.0, 0.0, W_R[2];   

        P << W_P[0], 0.0, 0.0,
             0.0, W_P[1], 0.0,
             0.0, 0.0, W_P[2]; 

        N << W_N[0], 0.0, 0.0,
             0.0, W_N[1], 0.0,
             0.0, 0.0, W_N[2]; 

        A << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;

        C << 1.0, 0.0, 0.0,
            0.0, 1.0, -1.0,
            1.0, 0.0, -1.0;

        B.setZero();

        w_til_des.resize(NP * nx);
        w_til_ref.resize(NP * nx);
        y_til_ref.resize(NP * nx);

        U_opt.resize(NC*nu);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group = cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcHlcNode::param_callback, this, _1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(int(Ts * 1000.0)),
            std::bind(&MpcHlcNode::calculateHighLevelController, this), cb_group_obs_);

        // Suscripciones

        subscriber_state = this->create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state", 1,
            std::bind(&MpcHlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        subscriber_ref_hlc_ = this->create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_hlc", 1, std::bind(&MpcHlcNode::callbackRefHlc,
                this, std::placeholders::_1), options_sensors_);

        subscriber_error_mlc_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/" + my_id + "/control/error_mlc", 1, std::bind(&MpcHlcNode::callbackerrorMlc,
                this, std::placeholders::_1), options_sensors_);

        subscriber_mlc_slave = this->create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/comunication/mlc_slave", 1, std::bind(&MpcHlcNode::callbackMlcSlave,
                this, std::placeholders::_1), options_sensors_);        

        publisher_w_virtual_ = this->create_publisher<std_msgs::msg::Float64>("/" + my_id + "/control/w_virtual", 1);

        publisher_ref_master_ = this->create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/ref_master", 10);

        RCLCPP_INFO(this->get_logger(), "High Level Controller MPC Node in %s has been started.", my_id.c_str());

    }

private:
    void calculateHighLevelController()
    {
        if (armed == false) {
            count=0.0;
            w_v = 0.0;
            w_m = 0.0;
            w_s = 0.0;

            Uf_v = 0.4;
            Uf_m = 0.4;
            Uf_s = 0.4;
            U_opt.setZero();
            {
                std::lock_guard<std::mutex> lock(mutex_);
                U_f = 0.4;
                w_m = 0.0;
                w_s = 0.0;
            }
        }
        else {
            if (count > -1) {
                //auto start = std::chrono::high_resolution_clock::now();
                //RCLCPP_INFO(this->get_logger(), "Ejecutand2");
                auto msg = geometry_msgs::msg::Vector3();
                auto msg_w = std_msgs::msg::Float64();
                
                float U_f_i;
                float w_s_i;    
                float w_m_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    U_f_i = U_f;
                    w_m_i = w_m;
                    w_s_i = w_s;// w_m; simular un barco considerando al otro en la misma posicion
                }

                if(count==0){
                    //------Primera iteracion del bucle de control
                    w_til_des.setZero();
                    U_opt.setConstant(U_f_i); 
                    count++;
                }
                float vec_f_c[3];
                currentTarget(w_v, vec_f_c);

                Ud_ref << U_f_i,
                    U_f_i*(vec_f_c[0]/vec_f_c[1]),
                    U_f_i*(vec_f_c[0]/vec_f_c[2]);

                B << vec_f_c[0], 0.0, 0.0,
                    0.0, vec_f_c[1], 0.0,
                    0.0, 0.0, vec_f_c[2];   
                
                B *= Ts;

                for(int k = 0; k < NP; k++){
                    w_til_ref.segment(k * nx, nx) = A.cast<double>() * w_til_des.segment(k * nx, nx) + B.cast<double>() * Ud_ref.cast<double>();
                    y_til_ref.segment(k*ny, ny) = C.cast<double>() * w_til_ref.segment(k*ny, ny);
                }

                //------------------------------------------------------------------------------------//
                //                           CODIGO MPC HIHG LEVEL                                    //
                //------------------------------------------------------------------------------------//

                try {
                    // Inicializaci�n de Gurobi 
                    GRBEnv env = GRBEnv(true);
                    env.set("LogFile", "mpc_hlc.log");
                    env.start();
                    GRBModel model = GRBModel(env);

                    // Definir las Variables de optimizaci�n como vectores columna 
                    std::vector<GRBVar> y_til((NP + 1) * ny);   // vector de los salidas del sistema a lo largo del horizonte de predicci�n
                    std::vector<GRBVar> w_til((NP + 1) * nx); // vector de los estados del sistema a lo largo del horizonte de predicci�n
                    std::vector<GRBVar> Ud_til(NC * nu);        // vector de las entradas del sistema a lo largo del horizonte de control

                    // Definir variables de estado inicial
                    y_til[0] = model.addVar(w_v, w_v, 0.0, GRB_CONTINUOUS, "y_v_0");
                    y_til[1] = model.addVar(w_m_i-w_s_i, w_m_i-w_s_i, 0.0, GRB_CONTINUOUS, "y_m_0");
                    y_til[2] = model.addVar(w_m_i-w_s_i, w_m_i-w_s_i, 0.0, GRB_CONTINUOUS, "y_s_0");
                    
                    // Definir variables de estado a lo largo del horizonte de predicci�n
                    for (int k = 1; k <= NP; ++k) {
                        y_til[k * ny + 0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_v_" + std::to_string(k));
                        y_til[k * ny + 1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_m_" + std::to_string(k));
                        y_til[k * ny + 2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "y_s_" + std::to_string(k));
                    }

                    // Definir variables de estado inicial
                    w_til[0] = model.addVar(w_v, w_v, 0.0, GRB_CONTINUOUS, "w_v_0");
                    w_til[1] = model.addVar(w_m_i, w_m_i, 0.0, GRB_CONTINUOUS, "w_m_0");
                    w_til[2] = model.addVar(w_s_i, w_s_i, 0.0, GRB_CONTINUOUS, "w_s_0");

                    // Definir variables de estado a lo largo del horizonte de predicci�n
                    for (int k = 1; k <= NP; ++k) {
                        w_til[k * nx + 0] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "w_v_" + std::to_string(k));
                        w_til[k * nx + 1] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "w_m_" + std::to_string(k));
                        w_til[k * nx + 2] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "w_s_" + std::to_string(k));
                    }

                    // Definir variables de control a lo largo del horizonte de control
                    for (int k = 0; k < NC; ++k) {
                        Ud_til[k * nu + 0] = model.addVar(Ud_min[0], Ud_max[0], 0.0, GRB_CONTINUOUS, "Ud_til_v_" + std::to_string(k));
                        Ud_til[k * nu + 1] = model.addVar(Ud_min[1], Ud_max[1], 0.0, GRB_CONTINUOUS, "Ud_til_m_" + std::to_string(k));
                        Ud_til[k * nu + 2] = model.addVar(Ud_min[2], Ud_max[2], 0.0, GRB_CONTINUOUS, "Ud_til_s_" + std::to_string(k));

                        // Asignar valores iniciales
                        Ud_til[k * nu + 0].set(GRB_DoubleAttr_Start, U_opt[k * nu + 0]);  // Valor inicial para Ud_til_v
                        Ud_til[k * nu + 1].set(GRB_DoubleAttr_Start, U_opt[k * nu + 1]);  // Valor inicial para Ud_til_m
                        Ud_til[k * nu + 2].set(GRB_DoubleAttr_Start, U_opt[k * nu + 2]);  // Valor inicial para Ud_til_s
                    }

                    // -----------------------------------------------------------------------------------------//
                    //                                    FUNCI�N OBJETIVO                                      //
                    // -----------------------------------------------------------------------------------------//
                    
                    GRBQuadExpr objective = 0;

                    // Penalizaci�n del Error de Seguimiento de las salidas 
                    // (y(k+1) - yref(k+1))'*Q*(y(k+1) - yref(k+1))

                    for (int k = 0; k < NP; ++k) {
                        objective += (y_til[(k + 1) * ny + 0] - y_til_ref[k * ny + 0]) * Q(0, 0) * (y_til[(k + 1) * ny + 0] - y_til_ref[k * ny + 0]);
                        objective += (y_til[(k + 1) * ny + 1] - y_til_ref[k * ny + 1]) * Q(1, 1) * (y_til[(k + 1) * ny + 1] - y_til_ref[k * ny + 1]);
                        objective += (y_til[(k + 1) * ny + 2] - y_til_ref[k * ny + 2]) * Q(2, 2) * (y_til[(k + 1) * ny + 2] - y_til_ref[k * ny + 2]);
                    }

                    // Penalizaci�n del coste terminal  
                    // (y(NP) - yref(NP))'*P*(y(NP) - yref(NP))

                    objective += (y_til[NP * ny + 0] - y_til_ref[(NP - 1) * ny + 0]) * P(0, 0) * (y_til[NP * ny + 0] - y_til_ref[(NP - 1) * ny + 0]);
                    objective += (y_til[NP * ny + 1] - y_til_ref[(NP - 1) * ny + 1]) * P(1, 1) * (y_til[NP * ny + 1] - y_til_ref[(NP - 1) * ny + 1]);
                    objective += (y_til[NP * ny + 2] - y_til_ref[(NP - 1) * ny + 2]) * P(2, 2) * (y_til[NP * ny + 2] - y_til_ref[(NP - 1) * ny + 2]);

                    for (int k = 0; k < NC; ++k) {

                        // Penalizai�n de las acciones de control
                        // (u(k)'*R*u(k)
                        objective += Ud_til[k * nu + 0] * R(0, 0) * Ud_til[k * nu + 0];
                        objective += Ud_til[k * nu + 1] * R(1, 1) * Ud_til[k * nu + 1];
                        objective += Ud_til[k * nu + 2] * R(2, 2) * Ud_til[k * nu + 2];

                        // Penalizaci�n del Error de Seguimiento de las entradas 
                        // (u(k) - uref(k))'*N*(u(k) - uref(k))
                        objective += (Ud_til[k * nu + 0] - Ud_ref[0]) * N(0, 0) * (Ud_til[k * nu + 0] - Ud_ref[0]);
                        objective += (Ud_til[k * nu + 1] - Ud_ref[1]) * N(1, 1) * (Ud_til[k * nu + 1] - Ud_ref[1]);
                        objective += (Ud_til[k * nu + 2] - Ud_ref[2]) * N(2, 2) * (Ud_til[k * nu + 2] - Ud_ref[2]);
                    }

                    model.setObjective(objective, GRB_MINIMIZE);

                    // -----------------------------------------------------------------------------------------//
                    //                                  RESTRICCIONES  
                    // -----------------------------------------------------------------------------------------// 

                    // Restricciones salidas a lo largo del horizonte de predicci�n 
                    for (int k = 0; k < NP; ++k) {
                        std::vector<GRBLinExpr> y_next(ny), w_next(nx);
                        Modelo(model, w_next, y_next, { w_til[k * nx], w_til[k * nx + 1], w_til[k * nx + 2] },
                                { Ud_til[k * nu], Ud_til[k * nu + 1], Ud_til[k * nu + 2]});

                        // ----------------- Restricciones de actualizaci�n de estados --------------------------
                        // Con este bucle estamos imponiendo que x(k+1) se igual a x_next (es decir los estados siguientes calculados con la funci�n modelo).

                        for (int i = 0; i < nx; ++i) {
                            model.addQConstr(w_til[(k + 1) * nx + i] == w_next[i]);
                            model.addQConstr(y_til[(k + 1) * ny + i] == y_next[i]); // mismo bucle porque nx es = ny
                        }
                    }

                    // -----------------------------------------------------------------------------------------// 
                    // Restricciones entradas a lo largo del horizonte de control
                    for (int k = 0; k < NC; ++k) {
                        // ------------------ Restricciones de incremento de las entradas ---------------------------

                        if (k == 0) {
                            model.addConstr(Ud_til[0] - Uf_v >= Delta_Ud_min[0]);
                            model.addConstr(Ud_til[0] - Uf_v <= Delta_Ud_max[0]);
                            model.addConstr(Ud_til[1] - Uf_m >= Delta_Ud_min[1]);
                            model.addConstr(Ud_til[1] - Uf_m <= Delta_Ud_max[1]);
                            model.addConstr(Ud_til[2] - Uf_s >= Delta_Ud_min[2]);
                            model.addConstr(Ud_til[2] - Uf_s <= Delta_Ud_max[2]);
                        }
                        else {
                            model.addConstr(Ud_til[k * nu] - Ud_til[(k - 1) * nu] >= Delta_Ud_min[0]);
                            model.addConstr(Ud_til[k * nu] - Ud_til[(k - 1) * nu] <= Delta_Ud_max[0]);
                            model.addConstr(Ud_til[k * nu + 1] - Ud_til[(k - 1) * nu + 1] >= Delta_Ud_min[1]);
                            model.addConstr(Ud_til[k * nu + 1] - Ud_til[(k - 1) * nu + 1] <= Delta_Ud_max[1]);
                            model.addConstr(Ud_til[k * nu + 2] - Ud_til[(k - 1) * nu + 2] >= Delta_Ud_min[2]);
                            model.addConstr(Ud_til[k * nu + 2] - Ud_til[(k - 1) * nu + 2] <= Delta_Ud_max[2]);
                        }
                    }

                    // -----------------------------------------------------------------------------------------//
                    //        OPTIMIZACI�N DEL MODELO Y APLICACI�N DE LAS MEJORES ACCIONES DE CONTROL           //
                    // -----------------------------------------------------------------------------------------// 
                    // Optimizar el modelo
                    model.optimize();

                    // Verificaci�n del cumplimiento de restricciones
                    if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
                        RCLCPP_ERROR(this->get_logger(), "El modelo es infeasible, generando reporte IIS...");
                        // Computar el IIS (Infeasibility Irreducible Subsystem)
                        model.computeIIS();
                        // Guardar el reporte IIS en un archivo
                        model.write("infeasibility_report.ilp");
                        // Asigna el valor de la flota a las salidas en caso de error o sin solucion
                        U_opt.setConstant(U_f_i); 
                    }
                    else if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL || model.get(GRB_IntAttr_Status) == GRB_SUBOPTIMAL) {
                        // Si el modelo encontr� una soluci�n �ptima o sub�ptima

                        // Extraer las soluciones y aplicarlas al sistema
                        for (int k = 0; k < NC; ++k) {
                            U_opt[k * nu + 0] = Ud_til[k * nu + 0].get(GRB_DoubleAttr_X);
                            U_opt[k * nu + 1] = Ud_til[k * nu + 1].get(GRB_DoubleAttr_X);
                            U_opt[k * nu + 2] = Ud_til[k * nu + 2].get(GRB_DoubleAttr_X);
                            // RCLCPP_INFO(this->get_logger(), "Ud_til[%d] = (%f, %f, %f)", k, U_opt[k * nu + 0], U_opt[k * nu + 1], U_opt[k * nu + 2]);
                        }
                    }
                    else {
                        RCLCPP_ERROR(this->get_logger(), "Sin solucion optima, generando reporte IIS...");
                        model.computeIIS();
                        // Guardar el reporte IIS en un archivo
                        model.write("infeasibility_report.ilp");
                        // Asigna el valor de la flota a las salidas en caso de error o sin solucion
                        U_opt.setConstant(U_f_i); 
                    }
                }
                // -------------------- Manejo de excepciones con Gurobi ---------------------------------- // 
                catch (GRBException& e) {
                    RCLCPP_ERROR(this->get_logger(), "Error code = %d, %s", e.getErrorCode(), e.getMessage().c_str());
                    // Asigna el valor de la flota a las salidas en caso de error o sin solucion
                    U_opt.setConstant(U_f_i); 
                }
                catch (...) {
                    RCLCPP_ERROR(this->get_logger(), "Exception during optimization");
                    // Asigna el valor de la flota a las salidas en caso de error o sin solucion
                    U_opt.setConstant(U_f_i);   
                }

                // -----------------------------------------------------------------------------------------// 
                // Aplicar las soluciones �ptimas
                Uf_v = U_opt[0];
                Uf_m = U_opt[1];
                Uf_s = U_opt[2];
                // Dezplazar la ultima solucion optima para la siguiente iteracion
                U_opt.head((NC*nu)- nu) = U_opt.segment(nu, (NC*nu)- nu);
                U_opt.tail(nu) = U_opt.segment((NC*nu)- nu - nu, nu);

                msg.x = Uf_v;
                msg.y = Uf_m;
                msg.z = Uf_s;    
                publisher_ref_master_->publish(msg);

                double e_slave = abs(w_til_ref[2]-w_s_i);
                Eigen::Vector3d e_ast(e_slave, e_slave, e_slave);

                w_til_des.segment(0, nx) = w_til_des.segment(nx, nx); //asigno segundo valor al primeroa como semilla

                for(int k = 1; k < NP; k++){
                    w_til_des.segment(k * nx, nx) = A.cast<double>() * w_til_des.segment((k-1) * nx, nx) + B.cast<double>() * Ud_ref.cast<double>() + C_1 * e_ast;
                }
                msg_w.data = w_v;
                publisher_w_virtual_->publish(msg_w);
                w_v = w_v + Ts * vec_f_c[0] * U_f_i;
                
                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start;
                // double miliseconds = elapsed.count()*1000;
                // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
            }
            else {
                count++;
            }
        }
    }

    //--------------------------------------------------------------------------------------------------------------------//
    //                                       FUNCIÓN MODELO Y LEY PROPORCIONAL 
    //--------------------------------------------------------------------------------------------------------------------//

    void Modelo(GRBModel& model, std::vector<GRBLinExpr>& w_next, std::vector<GRBLinExpr>& y_next, const std::vector<GRBVar>& w, 
                const std::vector<GRBVar>& u)
    {

        for (int i = 0; i < nx; ++i) {
            w_next[i] = 0;
            for (int j = 0; j < nx; ++j) {
                w_next[i] += A(i, j) * w[j] + B(i, j) * u[j];
            }
        }

        for (int i = 0; i < ny; ++i) {
            y_next[i] = 0;
            for (int j = 0; j < nx; ++j) {
                y_next[i] += C(i, j) * w_next[j];
            }
        }
    }

    //--------------------------------------------------------------------------------------------------------------------//
    void callbackRefHlc(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            U_f = msg->data;
        }
    }

    void callbackMlcSlave(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            w_s = msg->data;
        }
    }

    void callbackerrorMlc(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            w_m = msg->z; // Ultimo dato del error del medio nivel
        }
    }


    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed = msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    void currentTarget(float w, float* vec){
        /* 
        vec[0] -> FACTOR CURVATURA VEHICULO VIRTUAL
        vec[1] -> FACTOR CURVATURA VEHICULO MASTER
        vec[2] -> FACTOR CURVATURA VEHICULO SLAVE
        */
        switch(circuit) {
            case 0:
                // Linea recta en culquier dirreccion
                vec[0] = 1;
                vec[1] = 1;
                vec[2] = 1;
                break;
            case 1:
                vec[0] = curva_ala_1_4(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_1_6(w).f_c;
                    vec[2] = curva_ala_1_2(w).f_c;
                }else{
                    vec[1] = curva_ala_1_2(w).f_c;
                    vec[2] = curva_ala_1_6(w).f_c;
                }
                break;
            case 2:
                vec[0] = curva_ala_2_4(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_2_6(w).f_c;
                    vec[2] = curva_ala_2_2(w).f_c;
                }else{
                    vec[1] = curva_ala_2_2(w).f_c;
                    vec[2] = curva_ala_2_6(w).f_c;
                } 
                break;
            case 3:
                vec[0] = curva_ala_3_2(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_3_3(w).f_c;
                    vec[2] = curva_ala_3_1(w).f_c;
                }else{
                    vec[1] = curva_ala_3_1(w).f_c;
                    vec[2] = curva_ala_3_3(w).f_c;
                } 
                break;
            case 4:
                vec[0] = curva_ala_4_3(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_4_4(w).f_c;
                    vec[2] = curva_ala_4_2(w).f_c;
                }else{
                    vec[1] = curva_ala_4_2(w).f_c;
                    vec[2] = curva_ala_4_4(w).f_c;
                } 
                break;
            case 5:
                vec[0] = curva_ala_5_5(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_5_6(w).f_c;
                    vec[2] = curva_ala_5_4(w).f_c;
                }else{
                    vec[1] = curva_ala_5_4(w).f_c;
                    vec[2] = curva_ala_5_6(w).f_c;
                } 
                break;
            case 6:
                vec[0] = curva_ala_6_3(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_6_4(w).f_c;
                    vec[2] = curva_ala_6_2(w).f_c;
                }else{
                    vec[1] = curva_ala_6_2(w).f_c;
                    vec[2] = curva_ala_6_4(w).f_c;
                } 
                break;
            case 7:
                vec[0] = curva_ala_7_5(w).f_c;            // Factor de Curvatura
                if(path_max){
                    vec[1] = curva_ala_7_6(w).f_c;
                    vec[2] = curva_ala_7_4(w).f_c;
                }else{
                    vec[1] = curva_ala_7_4(w).f_c;
                    vec[2] = curva_ala_7_6(w).f_c;
                } 
                break;
        }
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto& param : params) {
            if (param.get_name() == "NC") {
                if (param.as_int() >= 1 and param.as_int() < 50) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NC = param.as_int();
                    U_opt.resize(NC*nu);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "NP") {
                if (param.as_int() >= 1 and param.as_int() < 50) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    NP = param.as_int();
                    w_til_des.resize(NP * nx);
                    w_til_ref.resize(NP * nx);
                    y_til_ref.resize(NP * nx);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-30");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "C_1") {
                if (param.as_double() >= 0.0 and param.as_double() < 2.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    C_1 = param.as_double();
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0.0-2.00");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Ts") {
                if (param.as_double() > 0.0 and param.as_double() < 10.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    //Inicializo el timer con el nuevo Ts
                    Ts = param.as_double();
                    if (timer_) {
                        timer_->cancel();
                    }
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(int(Ts * 1000.0)),
                                    std::bind(&MpcHlcNode::calculateHighLevelController, this), cb_group_obs_);
                }
                else {
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0.1-10.00");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_Q"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> W_Q = param.as_double_array();
                    Q << W_Q[0], 0.0, 0.0,
                         0.0, W_Q[1], 0.0,
                         0.0, 0.0, W_Q[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_R"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> W_R = param.as_double_array();
                    R << W_R[0], 0.0, 0.0,
                         0.0, W_R[1], 0.0,
                         0.0, 0.0, W_R[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_P"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> W_P = param.as_double_array();
                    P << W_P[0], 0.0, 0.0,
                         0.0, W_P[1], 0.0,
                         0.0, 0.0, W_P[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "W_N"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    std::vector<double> W_N = param.as_double_array();
                    N << W_N[0], 0.0, 0.0,
                         0.0, W_N[1], 0.0,
                         0.0, 0.0, W_N[2];
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "circuit"){
                if(param.as_int() >= 0 and param.as_int() <= 7){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    circuit = param.as_int();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "path_max"){
                RCLCPP_INFO(this->get_logger(), "changed param value");
                path_max = param.as_bool();
            }
            if (param.get_name() == "Ud_min"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Ud_min = param.as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Ud_max"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Ud_max = param.as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }    
            if (param.get_name() == "Delta_Ud_min"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_Ud_min = param.as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "Delta_Ud_max"){
                if(param.as_double_array().size() == 3){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Delta_Ud_max = param.as_double_array();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change parameter value, array size must be 3");
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

    //------Params-------//
    float Ts;
    int count = 0, circuit;
    bool path_max;

    //Par�metros del controlador MPC
    int NC, NP;
    double C_1;

    // Definicion Matrices y Vectores

    Matrix <float, 3,3> Q;
    Matrix <float, 3,3> R;
    Matrix <float, 3,3> P;
    Matrix <float, 3,3> N;
    Matrix <float, 3,3> A;
    Matrix <float, 3,3> B;
    Matrix <float, 3,3> C;
    Vector <float, 3> Ud_ref;
    std::vector <double> Ud_min; 
    std::vector <double> Ud_max; 

    std::vector <double> Delta_Ud_min; 
    std::vector <double> Delta_Ud_max; 

    VectorXd w_til_des;
    VectorXd w_til_ref;
    VectorXd y_til_ref;
    VectorXd U_opt;

    float w_v = 0.0, U_f = 0.0, w_m = 0.0, w_s = 0.0;
    float Uf_v = 0.0, Uf_m = 0.0, Uf_s=0.0;
     
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_error_mlc_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_ref_hlc_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_mlc_slave;
    
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_ref_master_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_w_virtual_;
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
    auto node = std::make_shared<MpcHlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}