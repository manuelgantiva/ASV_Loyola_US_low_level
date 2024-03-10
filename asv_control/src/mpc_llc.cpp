#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "asv_interfaces/msg/pwm_values.hpp"        //Interface pwm values override
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer

#include <cmath>
#include <thread>
#include <vector>
#include <complex>
#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>

#include <qpOASES.hpp>  
using namespace qpOASES;

using namespace std;
using namespace Eigen;

using std::placeholders::_1;

// Declaración de contantes
const int PWMMAX = 1900;
const int PWMMIN = 1900;

const float IGu_MAX = 0.0425;
const float IGu_MIN = 0.0;

const float IGr_MAX = 0.0929;
const float IGr_MIN = -0.0929;

//-----------------------------------------------------------------------------
// Declaración de contantes MPC 

// Dimensiones de las matrices

const int n_x = 4;
const int n_u = 2;
const int n_p = 3;
const int n_y = 2;

const float NOM_ref_U = 1;                            // Valor nominal de la referencia sobre la velocidad lineal (m/s)
const float NOM_ref_U_dot = 0;                        // Valor nominal de la referencia sobre la aceleración lineal (m/s2)
const float Max_Force = 10;                           // Máxima fuerza aplicable por parte de cada propulsor (N)


const VectorXd T_max = (VectorXd(n_u) << Max_Force, Max_Force).finished();
const VectorXd T_min = (VectorXd(n_u) << -0.5*Max_Force, -0.5*Max_Force).finished();

const VectorXd X_max = (VectorXd(n_x) << M_PI, 2.5 * NOM_ref_U, 2 * NOM_ref_U, 1e3).finished();
const VectorXd X_min = (VectorXd(n_x) << -M_PI, 0, -2 * NOM_ref_U, -1e3).finished();

// Parámetros del barco

const float m = 23.8;
const float Iz = 1.76;
const float xG = 0.046;

const float d = 1.5;

const float X_udot = -2;                                                 	// Parámetro hidrodinámico X_udot (kg)
const float Y_vdot = -10;                                                 	// Parámetro hidrodinámico Y_vdot (kg)
const float Y_rdot = -0;                                                 	// Parámetro hidrodinámico Y_rdot (kg m)
const float N_vdot = -0;                                                 	// Parámetro hidrodinámico N_vdot (kg m)
const float N_rdot = -1;                                                 	// Parámetro hidrodinámico N_rdot (kg m2)

const float m11 = m - X_udot;
const float m22 = m - Y_vdot;
const float m23 = m * xG - Y_rdot;
const float m32 = m * xG - N_vdot;
const float m33 = Iz - N_rdot;

const float g_u_dot = 1 / m11;
const float g_v_dot = -m23 / (m22 * m33 - m23 * m32);
const float g_r_dot = m22 / (m22 * m33 - m23 * m32);


//-----------------------------------------------------------------------------

class MpcLlcNode : public rclcpp::Node
{
public:
    MpcLlcNode() : Node("mpc_llc")
    {     
        // --------- Parámetros MPC --------- //
        
        this-> declare_parameter("Ts", 100.0);      // Tiempo de Muestreo

        this-> declare_parameter("W_u", 1e2);                          // Peso para los errores de seguimiento de la referencia de surge
        this-> declare_parameter("W_v", 1e0);                          // Peso para los errores de seguimiento de la referencia de sway
        this-> declare_parameter("W_psi", 1e2);                        // Peso para los errores de seguimiento de la referencia de heading
        this-> declare_parameter("W_r", 1e2);                          // Peso para los errores de seguimiento de la referencia de yaw

        this-> declare_parameter("W_R", 1e0);                          // Peso para la acción de control del propulsor derecho
        this-> declare_parameter("W_L", 1e0);                          // Peso para la acción de control del propulsor izquierdo

        this-> declare_parameter("lambda_terminalCost", 1.1e0);        // Peso del coste terminal en la función objetivo

        this-> declare_parameter("N_p", 10);
        this-> declare_parameter("N_c", 4);

        this-> declare_parameter("tau_f", 0);                          // Constante de tiempo para el filtro del error de predicción

        //-----------------------------------------------------------------------------
        //---------Parámetros del LLC-------------------//
        
        this-> declare_parameter("taud", 350); // Taud = #*Ts Est es # derivativo
        this-> declare_parameter("IGr_en", 1.0);    //Habilitar Input Gain R

        this-> declare_parameter("Xu6", -0.0166263484863104);
        this-> declare_parameter("Xu7", 0.0592216770505099);
        this-> declare_parameter("Xr11", 0.0127391511137106);
        this-> declare_parameter("Xr13", 0.190555832330969);

        //-----------------------------------------------------------------------------
        // ------------ obtener y guardar parametros MPC --------

        Ts = this->get_parameter("Ts").as_double()/1000.0;

        W_u = this->get_parameter("W_u").as_double();
        W_v = this->get_parameter("W_v").as_double();
        W_psi = this->get_parameter("W_psi").as_double();
        W_r = this->get_parameter("W_r").as_double();

        W_R = this->get_parameter("W_R").as_double();
        W_L = this->get_parameter("W_L").as_double();

        lambda_terminalCost = this->get_parameter("lambda_terminalCost").as_double();

        N_p = this->get_parameter("N_p").as_double();
        N_c = this->get_parameter("N_c").as_double();

        tau_f = this->get_parameter("tau_f").as_double();

        //-----------------------------------------------------------------------------
        // ----------------------------------------------------------------------------

        taud = this->get_parameter("taud").as_int();
        IGr_en = this->get_parameter("IGr_en").as_double();

        Xu6 = this->get_parameter("Xu6").as_double();
        Xu7 = this->get_parameter("Xu7").as_double();
        Xr11 = this->get_parameter("Xr11").as_double();
        Xr13 = this->get_parameter("Xr13").as_double();

        memory_u.assign(4, 0.0);
        memory_r.assign(4, 0.0);

        a=(taud*Ts)/(taud*Ts+Ts);
        b=1/(taud*Ts+Ts);

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&MpcLlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/control/state_observer",rclcpp::SensorDataQoS(), std::bind(&MpcLlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<geometry_msgs::msg::Vector3>(
            "/control/reference_llc", 1, std::bind(&MpcLlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/mavros/state",1,
                std::bind(&MpcLlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_pwm = this-> create_publisher<asv_interfaces::msg::PwmValues>("/control/pwm_value_ifac",
                rclcpp::SensorDataQoS());
        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MpcLlcNode::calculateLowLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Low Level Controller IFAC Node has been started.");
    	
    }

private:
    void calculateLowLevelController()
    {
        if(armed==false){
            c_ref=0;
            memory_u.assign(4, 0.0);
            memory_r.assign(4, 0.0);
            count=0;
        }else{
            //auto start = std::chrono::high_resolution_clock::now();
            auto msg = asv_interfaces::msg::PwmValues();

            float u_hat_i;
            float v_hat_i;
            float r_hat_i;
            float psi_hat_i;

            float sig_u_i;
            float sig_v_i;
            float sig_r_i;

            float u_ref_i;
            float r_ref_i;
            float psi_ref_i;

            // No es observador

            float psi_est_ant_i;
            float u_est_ant_i;
            float v_est_ant_i;
            float r_est_ant_i;

            float T_R_ant_i; 
            float T_L_ant_i;

            float T_R_ant_est_i;
            float T_L_ant_est_i;

            VectorXd Sol_Delta_T_ant_i;        
            VectorXd factor_correccion_shifted_i;
            VectorXd factor_correccion_filtrado_i;
        
            {
                std::lock_guard<std::mutex> lock(mutex_);

                u_hat_i=u_hat;
                v_hat_i=v_hat;
                r_hat_i=r_hat;
                psi_hat_i=psi_hat;

                sig_u_i=sig_u;
                sig_v_i=sig_v;
                sig_r_i=sig_r;

                u_ref_i=u_ref;
                r_ref_i=r_ref;
                psi_ref_i=psi_ref;

                // No es observador

                T_R_ant_i = T_R_ant;
                T_L_ant_i = T_L_ant;

                psi_est_ant_i = psi_est_ant;
                u_est_ant_i = u_est_ant;
                v_est_ant_i = v_est_ant;
                r_est_ant_i = r_est_ant;

                T_R_ant_est_i = T_R_ant_est;
                T_L_ant_est_i = T_L_ant_est;
 
                Sol_Delta_T_ant_i = Sol_Delta_T_ant;
                factor_correccion_shifted_i = factor_correccion_shifted;
                factor_correccion_filtrado_i = factor_correccion_filtrado; 
            }

            //Codigo MPC

            //----------------------------------------------------------------------------------------------------------------
            // Construción de vectores y matrices:

            VectorXd u_ref_vec = VectorXd::Constant(N_p, u_ref_i);
            VectorXd psi_ref_vec = VectorXd::Constant(N_p, psi_ref_i);
            VectorXd r_ref_vec = VectorXd::Constant(N_p, r_ref_i);
            VectorXd v_ref_vec = VectorXd::Zero(N_p); 
            VectorXd T_ant(2);

            T_ant << T_R_ant_i, T_L_ant_i;

            MatrixXd factor_correccion = MatrixXd::Zero(n_u + n_x, 2);
 
            for(int i = 0; i < n_u + n_x; ++i) {
                factor_correccion(i, 0) = factor_correccion_shifted_i(i);
                factor_correccion(i, 1) = factor_correccion_shifted_i(i + n_u + n_x);
            }

            //----------------------------------------------------------------------------------------------------------------

            // 4. CÁLCULO DE LAS MATRICES DEL MODELO LINEAL: 

            // Definición de A

            MatrixXd A = MatrixXd::Identity(n_x, n_x);
            A(0, 3) = Ts;

            // Definición de B
           
            MatrixXd B(n_x, n_u);

            B << 0, 0,
                g_u_dot * Ts, g_u_dot * Ts,
                -g_v_dot * Ts * d / 2, g_v_dot * Ts * d / 2,
                -g_r_dot * Ts * d / 2, g_r_dot * Ts * d / 2;

            // Definición de B_d

            MatrixXd B_d = MatrixXd::Zero(n_x, n_p);
            B_d(1, 0) = Ts;
            B_d(2, 1) = Ts;
            B_d(3, 2) = Ts;

            // Definición de C

            MatrixXd C = MatrixXd::Zero(n_y, n_x);
            C.block(0, 0, n_y, n_y) = MatrixXd::Identity(n_y, n_y);

            // Modelo aumentado:

            MatrixXd A_til(n_x + n_u, n_x + n_u);
            A_til.block(0, 0, n_x, n_x) = A;
            A_til.block(0, n_x, n_x, n_u) = B;
            A_til.block(n_x, 0, n_u, n_x) = MatrixXd::Zero(n_u, n_x);
            A_til.block(n_x, n_x, n_u, n_u) = MatrixXd::Identity(n_u, n_u);

            MatrixXd B_til(n_x + n_u, n_u);
            B_til.block(0, 0, n_x, n_u) = B;
            B_til.block(n_x, 0, n_u, n_u) = MatrixXd::Identity(n_u, n_u);

            MatrixXd B_d_til(n_x + n_u, n_p);
            B_d_til.block(0, 0, n_x, n_p) = B_d;
            B_d_til.block(n_x, 0, n_u, n_p) = MatrixXd::Zero(n_u, n_p);

            MatrixXd C_til(C.rows(), n_x + n_u);
            C_til.block(0, 0, C.rows(), n_x) = C;
            C_til.block(0, n_x, C.rows(), n_u) = MatrixXd::Zero(C.rows(), n_u);


            //----------------------------------------------------------------------------------------------------------------
            // 5. CÁLCULO DEL FACTOR DE CORRECCIÓN POR ERRORES DE PREDICCIÓN:

            // 5.1. Error de estimación en las salidas estimadas en el instante anterior:

            VectorXd X(6); 
            X << psi_hat_i, u_hat_i, v_hat_i, r_hat_i, T_R_ant_i, T_L_ant_i;

            VectorXd X_est_ant(6);
            X_est_ant << psi_est_ant_i, u_est_ant_i, v_est_ant_i, r_est_an_it, T_R_ant_est_i, T_L_ant_est_i;

            VectorXd e = X - X_est_ant;

            // 5.2. Cálculo del factor de corrección:

            float a = 0;
            float fd = pow(a, 2); // Usando pow para elevar a al cuadrado.
            float ki = 1 + fd - 2 * a;

            VectorXf fc = (1 + fd) * factor_correccion.col(0) - fd * factor_correccion.col(1) + e * ki;

            // 5.3. Filtro del factor de corrección

            VectorXd factor_correccion_filtrado_i = tau_f / (Ts + tau_f) * factor_correccion_filtrado_i + Ts / (Ts + tau_f) * fc;

            // 5.4. Actualización de variables del filtro de corrección

            factor_correccion.col(1) = factor_correccion.col(0);
            factor_correccion.col(0) = fc;

            // 5.5. Shifteado del factor de corrección en un solo vector

            VectorXd factor_correccion_shifted_i(2 * factor_correccion.rows());
            factor_correccion_shifted_i << factor_correccion.col(0), factor_correccion.col(1);

            //----------------------------------------------------------------------------------------------------------------
            // 6. CÁLCULO DE LA RESPUESTA LIBRE:

            MatrixXd F = MatrixXd::Zero((n_u + n_x) * N_p, n_x + n_u);
            MatrixXd A_til_pow = MatrixXd::Identity(n_x + n_u, n_x + n_u); 
            for (int i = 0; i < N_p; ++i) {
                A_til_pow = A_til_pow * A_til; // Calcula la i+1-ésima potencia de A_til
                F.block((n_u + n_x) * i, 0, n_u + n_x, n_x + n_u) = A_til_pow;
            }

            VectorXd X_REF = VectorXd::Zero((n_u + n_x) * N_p);
            VectorXd SIGMA = VectorXd::Zero(n_p * N_p);
            VectorXd FACTOR_CORRECCION = VectorXd::Zero((n_u + n_x) * N_p);

            for (int i = 0; i < N_p; ++i) {
                VectorXd T_eq = B.colPivHouseholderQr().solve((MatrixXd::Identity(n_x, n_x) - A) * (VectorXd(4) << psi_ref_vec[i], v_ref_vec[i], u_ref_vec[i], r_ref_vec[i]).finished() - B_d * VectorXd::Constant(3, 1, sig_u_i, sig_v_i, sig_r_i));
                X_REF.segment((n_u + n_x) * i, (n_u + n_x)) = (VectorXd(6) << psi_ref_vec[i], u_ref_vec[i], v_ref_vec[i], r_ref_vec[i], T_eq(0), T_eq(1)).finished();
                SIGMA.segment(n_p * i, n_p) = VectorXd::Constant(3, 1, sig_u_i, sig_v_i, sig_r_i);
                FACTOR_CORRECCION.segment((n_u + n_x) * i, (n_u + n_x)) = factor_correccion_filtrado_i;
            }

            MatrixXd H_d = MatrixXd::Zero((n_u + n_x) * N_p, n_p * N_p);

            for (int k = 0; k < N_p; ++k) {
                for (int i = k; i < N_p; ++i) {
                    MatrixXd A_til_pow_k = MatrixXd::Identity(n_x + n_u, n_x + n_u);
                    for (int j = 0; j < i - k; ++j) {
                        A_til_pow_k *= A_til; // Calcula A_til^(i-k) directamente
                    }
                    H_d.block((n_u + n_x) * i, n_p * k, n_u + n_x, n_p) = A_til_pow_k * B_d_til;
                }
            }
            
            VectorXd X_free = F * X + H_d * SIGMA + FACTOR_CORRECCION;

            //----------------------------------------------------------------------------------------------------------------
            // 7. CÁLCULO DE LA RESPUESTA FORZADA:

            MatrixXd H = MatrixXd::Zero((n_u + n_x) * N_p, n_u * N_c);

            for (int j = 0; j < N_c; ++j) {
                for (int i = j; i < N_p; ++i) {
                    MatrixXd A_til_pow_d = MatrixXd::Identity(n_x + n_u, n_x + n_u);
                    // Calcula A_til^(i-j)
                    for (int k = 0; k < i - j; ++k) {
                        A_til_pow_d *= A_til;
                    }
                    H.block((n_u + n_x) * i, n_u * j, n_u + n_x, n_u) = A_til_pow_d * B_til;
                }
            }

            //----------------------------------------------------------------------------------------------------------------
            // 8. RESTRICCIONES

            // Definir las matrices:

            MatrixXd A_constr = MatrixXd::Zero(2*n_u*N_c + 2*n_u*N_c, n_u*N_c);
            VectorXd b_constr = VectorXd::Zero(2*n_u*N_c + 2*n_u*N_c);

            MatrixXd A_constr_out = MatrixXd::Zero(2*n_u*N_c + 2*n_u*N_c + (n_u+n_x)*N_p + (n_u+n_x)*N_p, n_u*N_c);
            VectorXd b_constr_out = VectorXd::Zero(2*n_u*N_c + 2*n_u*N_c + (n_u+n_x)*N_p + (n_u+n_x)*N_p);
            
            int ind_fin_f = 0;

            // 8.1. Límites de las acciones de control:

            // 8.1.1. Límite máximo

            for (int j = 0; j < N_c; ++j) {
                for (int k = 0; k < j + 1; ++k) {
                    A_constr.block(ind_fin_f + j * n_u, k * n_u, n_u, n_u) = MatrixXd::Identity(n_u, n_u);
                }
                b_constr.segment(ind_fin_f + j * n_u, n_u) = T_max - T_ant; 
            }

            ind_fin_f = n_u * N_c;

            // 8.1.2. Límite mínimo

            for (int j = 0; j < N_c; ++j) {
                for (int k = 0; k < j + 1; ++k) {
                    A_constr.block(ind_fin_f + j * n_u, k * n_u, n_u, n_u) = -MatrixXd::Identity(n_u, n_u);
                }
                b_constr.segment(ind_fin_f + j * n_u, n_u) = -T_min + T_ant; 
            }

            ind_fin_f = 2 * n_u * N_c;
            
            // 8.2. Límites de los incrementos de acción de control:

            // 8.2.1. Límite máximo:
    
            A_constr.block(ind_fin_f, 0, n_u * N_c, n_u * N_c) = MatrixXd::Identity(n_u * N_c, n_u * N_c);
            for (int j = 0; j < N_c; ++j) {
                b_constr.segment(ind_fin_f + j * n_u, n_u) = Delta_T_max;
            }

            ind_fin_f = 2 * n_u * N_c + 1 * n_u * N_c;

            // 8.2.2.Límite mínimo:

            A_constr.block(ind_fin_f, 0, n_u * N_c, n_u * N_c) = -MatrixXd::Identity(n_u * N_c, n_u * N_c);
            for (int j = 0; j < N_c; ++j) {
                b_constr.segment(ind_fin_f + j * n_u, n_u) = -Delta_T_min;
            }

            ind_fin_f = 2 * n_u * N_c + 2 * n_u * N_c;

            // Actualizando A_constr_out y b_constr_out
            A_constr_out.topRows(ind_fin_f) = A_constr;
            b_constr_out.head(ind_fin_f) = b_constr;

            // 8.3. Límites de las variables de estado:

            // Límite máximo

            VectorXd X_MAX((n_u + n_x) * N_p);
            A_constr_out.block(ind_fin_f, 0, (n_u + n_x) * N_p, n_u * N_c) = H;

            for (int i = 0; i < N_p; ++i) {
                X_MAX.segment(i * (n_u + n_x), n_u + n_x) << X_max, T_max; // Concatenación de X_max y T_max
                b_constr_out.segment(ind_fin_f + i * (n_u + n_x), n_u + n_x) = X_MAX.segment(i * (n_u + n_x), n_u + n_x) - X_free.segment(i * (n_u + n_x), n_u + n_x);
            }
        
            ind_fin_f = 2 * n_u * N_c + 2 * n_u * N_c + (n_u + n_x) * N_p;

            // Límite mínimo

            VectorXd X_MIN((n_u + n_x) * N_p);
            A_constr_out.block(ind_fin_f, 0, (n_u + n_x) * N_p, n_u * N_c) = -H;
            for (int i = 0; i < N_p; ++i) {
                X_MIN.segment(i * (n_u + n_x), n_u + n_x) << X_min, T_min; 
                b_constr_out.segment(ind_fin_f + i * (n_u + n_x), n_u + n_x) = -X_MIN.segment(i * (n_u + n_x), n_u + n_x) + X_free.segment(i * (n_u + n_x), n_u + n_x);
            }
           
            //----------------------------------------------------------------------------------------------------------------

            //  9. CONSTRUCCIÓN DE LAS MATRICES DE PONDERACIÓN EN LA FUNCIÓN OBJETIVO: 

            //  9.1. Ponderación del error de seguimiento de referencias:

            MatrixXd Q = MatrixXd::Zero((n_u + n_x) * N_p, (n_u + n_x) * N_p);

            VectorXd Pesos_Q = VectorXd::Zero(n_u + n_x); 
            Pesos_Q << W_psi, W_u, W_v, W_r, 0, 0; 

            for (int i = 0; i < N_p; ++i) {
                Q.block(i*(n_u + n_x), i*(n_u + n_x), n_u + n_x, n_u + n_x) = Pesos_Q.asDiagonal();
            }

            // 9.2. Ponderación de los incrementos de las acciones de control:

            MatrixXd R = MatrixXd::Zero(n_u * N_c, n_u * N_c);
            VectorXd Pesos_R(n_u); 
            Pesos_R << W_R, W_L; 

            for (int j = 0; j < N_c; ++j) {
                R.block(j*n_u, j*n_u, n_u, n_u) = Pesos_R.asDiagonal();
            }

            // 9.3. Ponderación de la desviación de las acciones de control respecto al valor de referencia:

            MatrixXd N = MatrixXd::Identity(n_u * N_c, n_u * N_c);
            MatrixXd S = MatrixXd::Zero(n_u * N_c, n_u * N_c);
            VectorXd desv_u = VectorXd::Zero(n_u * N_c);


            VectorXd sp_u_val = B.colPivHouseholderQr().solve((MatrixXd::Identity(n_x, n_x) - A) * (VectorXd(4) << psi_ref_vec(0), v_ref_vec(0), u_ref_vec(0), r_ref_vec(0)).finished() - B_d * (VectorXd(3) << sig_u_i, sig_v_i, sig_r_i).finished());

            VectorXd sp_u = VectorXd::Zero(n_u * N_c);
            for (int k = 0; k < N_c; ++k) {
                for (int j = 0; j < n_u; ++j) {
                    sp_u(k * n_u + j) = sp_u_val(j);
                }
            }

            for (int i = 0; i < N_c; ++i) {
                for (int j = 0; j <= i; ++j) {
                    N.block(i * n_u, j * n_u, n_u, n_u) = MatrixXd::Identity(n_u, n_u);
                }
                S.block(i * n_u, i * n_u, n_u, n_u) = VectorXd((VectorXd(n_u) << W_R, W_L).finished()).asDiagonal();
                for (int j = 0; j < n_u; ++j) {
                    desv_u(i * n_u + j) = T_ant(j) - sp_u(i * n_u + j);
                }
            }

            // 9.4. Coste terminal:

            MatrixXd P = MatrixXd::Zero(6, 6);
            MatrixXd P_CT = MatrixXd::Zero((n_u + n_x) * N_p, (n_u + n_x) * N_p);

            P_CT.block((N_p - 1) * (n_u + n_x), (N_p - 1) * (n_u + n_x), n_u + n_x, n_u + n_x) = P;

            //--------------------------------------------------------------------------------------------------
            // 10. OBTENCIÓN DE LA ACCIÓN DE CONTROL:

            MatrixXd H_quadprog = H.transpose() * (Q + lambda_terminalCost * P_CT) * H + R + N.transpose() * S * N;

            VectorXd f_quadprog = H.transpose() * (Q + lambda_terminalCost * P_CT) * (X_free - X_REF) + N.transpose() * S * desv_u;

            // Definición del punto inicial:

            VectorXd Delta_T0 = VectorXd::Zero(n_u * N_c);

            // Comprobar si Sol_Delta_T_ant es diferente de cero
            if (!Sol_Delta_T_ant_i.isZero()) {
                // Copiar valores de Sol_Delta_T_ant, omitiendo el primer conjunto de controles
                Delta_T0.segment(0, n_u * (N_c - 1)) = Sol_Delta_T_ant_i.segment(n_u, n_u * (N_c - 1));

                // Establecer el último conjunto de controles según K_d * X
                Delta_T0.segment(n_u * (N_c - 1), n_u) = K_d * X;
            } else {
                // Si Sol_Delta_T_ant es cero, establecer todos los conjuntos de controles según K_d * X
                for (int j = 0; j < N_c; ++j) {
                    Delta_T0.segment(n_u * j, n_u) = K_d * X;
                }
            }

            // RESOLUCIÓN DEL PROBLEMA DE OPTIMIZACIÓN:

            int nV = n_u * N_c; // Número total de variables de decisión.
            int nC = A_constr_out.rows(); // Número de restricciones, derivado de las filas de A.

            QProblem MPC_Low_Level(nV, nC);
            
            // Configurar opciones

            Options options;
            options.setToMPC();  // Configuraciones recomendadas para MPC
            MPC_Low_Level.setOptions(options);

            // Convertir Eigen::MatrixXd y VectorXd a arreglos de dobles
            MatrixXd H_qp = H_quadprog;
            VectorXd g_qp = f_quadprog;
            MatrixXd A_qp = A_constr_out;
            
            // Para restricciones del tipo A*x <= b, lbA es -inf y ubA es b
            VectorXd lbA = -INFTY * VectorXd::Ones(nC); // -inf para todas las restricciones
            VectorXd ubA = b_constr_out; // Límites superiores son b

            VectorXd lb = -INFTY * VectorXd::Ones(nV); // -inf para x
            VectorXd ub = INFTY * VectorXd::Ones(nV); // inf para x

            // Inicializar el solver
            int nWSR = 100;  // número máximo de iteraciones
            MPC_Low_Level.init(H_qp.data(), g_qp.data(), A_qp.data(), lb.data(), ub.data(), lbA.data(), ubA.data(), nWSR, Delta_T0.data());

            // Obtener la solución
            VectorXd Delta_T(nV);
            MPC_Low_Level.getPrimalSolution(Delta_T.data());

            // SELECCIÓN DE LA ACCIÓN DE CONTROL EN EL INSTANTE ACTUAL:

            // Calcula el estado estimado.

            VectorXd X_est = H * Delta_T + X_free;

            // ESTOS VALORES VAN A SER LAS NUEVAS ENTRADAS

            // Extrae los componentes individuales del estado estimado.
            
            double psi_est = X_est(0); 
            double u_est = X_est(1);
            double v_est = X_est(2);
            double r_est = X_est(3);
            double T_R_ant_est = X_est(4);
            double T_L_ant_est = X_est(5);

            // actualizamos 

            psi_est_ant_i = psi_est;
            u_est_ant_i = u_est;
            v_est_ant_i = v_est;
            r_est_ant_i = r_est;
            T_R_ant_est_i = T_R_ant_est;
            T_L_ant_est_i = T_L_ant_est;

            // Calcula los nuevos controles T_R y T_L 

            double T_R = Delta_T(0) + T_R_ant_i; 
            double T_L = Delta_T(1) + T_L_ant_i;

            // Actualizacion 

            T_R_ant_i = T_R;
            T_L_ant_i = T_L;

            // Actualizacion factores de correccion y accion de control 

            Sol_Delta_T_ant_i = Delta_T;  
              
            factor_correccion_shifted_i = factor_correccion_shifted_i;
            factor_correccion_filtrado_i = factor_correccion_filtrado_i;

            //--------------------------------------------------------------------------------------------------
            float c_ref=r_ref_i-sm_gain_kpsi*(psi_hat_i-psi_ref_i);

            float IG_u = Ts*(u_dot_ref_i-sm_gain_ku*(u_hat_i-u_ref_i)-sig_u_i);
            float IG_r = IGr_en*Ts*(r_dot_ref_i-sm_gain_kr*(r_hat_i-c_ref)-sm_gain_kpsi*(r_hat_i-r_ref_i)-sig_r_i);

            //Codigo input Gain

            if(IG_u==0 && IG_r==0){
                msg.t_left=1500;
                msg.t_righ=1500;  
            }else{
                if(IG_u<IGu_MIN){
                    IG_u=IGu_MIN;
                }else if (IG_u > IGu_MAX) {
                    IG_u=IGu_MAX;
                }

                if(IG_r<IGr_MIN){
                    IG_r=IGr_MIN;
                }else if (IG_r > IGr_MAX) {
                    IG_r=IGr_MAX;
                }

                double t0 = -4 * Xr11 * Xr11;
                double t1 = -(4 * Xr11 * Xr11 * Xu7 / Xu6) - (4 * Xr11 * Xr13);
                double t2 = (4 * Xr11 * Xr11 * IG_u / Xu6) - (4 * Xr11 * Xr13 * Xu7 / Xu6) - (Xr13 * Xr13);
                double t3 = (4 * Xr11 * Xr13 * IG_u / Xu6) - (Xr13 * Xr13 * Xu7 / Xu6);
                double t4 = (Xr13 * Xr13 * IG_u / Xu6) - (IG_r * IG_r);


                vector<double> yr = solveY(t0, t1, t2, t3, t4);
                vector<double> res;

                for (double yi : yr) {
                    if (yi >= 0 && yi <= 1.13) {
                        res.push_back(yi);
                    }
                }

                double xf, yf;
                for (double yi : res) {
                    double xi = solveX(yi, IG_r, Xr11, Xr13);
                    xf = xi;
                    yf = yi;
                }

                double L, R;
                L = ((2 * yf + xf) / 2);
                if (L > 0) {
                    L = L + 0.0775;
                } else {
                    L = L - 0.0925;
                }

                R = ((2 * yf - xf) / 2);
                if (R > 0) {
                    R = R + 0.0775;
                } else {
                    R = R - 0.0925;
                }

                // Publish pwms
                msg.t_left=400 * L + 1500;
                msg.t_righ=400 * R + 1500;
                if(msg.t_left<1500){
                    msg.t_left=1500;
                }else if (msg.t_left > 1900) {
                    msg.t_left=1900;
                }
                if(msg.t_righ<1500){
                    msg.t_righ=1500;
                }else if (msg.t_righ > 1900) {
                    msg.t_righ=1900;
                }
                
            }
            if(count < 5){
                msg.t_left= 1500;
                msg.t_righ= 1500; 
                count=count+1;
            }
            publisher_pwm->publish(msg);
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // double miliseconds = elapsed.count()*1000;
            // Imprime el tiempo con dos decimales fijos
            // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
        }        
    }

    vector<double> filterRoots(const VectorXcd& roots) {
        vector<double> realRoots;
        for (int i = 0; i < roots.size(); ++i) {
            if (roots[i].imag() == 0) {
                double realPart = roots[i].real();
                realRoots.push_back(realPart);
            }
        }
        return realRoots;
    }

    vector<double> solveY(double t0, double t1, double t2, double t3, double t4) {
        VectorXcd coefficients(5);
        coefficients << t4, t3, t2, t1, t0;

        Eigen::PolynomialSolver<complex<double>, Eigen::Dynamic> solver;
        solver.compute(coefficients);

        return filterRoots(solver.roots());
    }

    double solveX(double y, double b, double c11, double c13) {
        double n = b / ((c11 * y) + (c13 / 2));
        return n;
    }

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

    void callbackVelReference(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
   
        float u_dot = derivationFilter(msg->x, memory_u, a, b);
        float r_dot = derivationFilter(msg->y, memory_r, a, b);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_ref = msg->x;
            r_ref = msg->y;
            u_dot_ref = u_dot;
            r_dot_ref = r_dot;
            psi_ref = msg->z;
        }
    }

    // Función para calcular la salida del filtro de derivación
    float derivationFilter(float input, std::vector<float>& memory, float a, float b){
        //float a1 = 0.0;
        //float b0 = 1.0;
        //float output = (b0 * input - f_diff * memory[0] + f_diff * memory[1] - a1 * memory[2] + f_diff * memory[3]) / f_diff;
        
        float output = (a * memory[0]) + b * (input - memory[1]);
        // Actualizar memoria para la próxima iteración
        memory[0] = output;
        memory[1] = input;

        return output;
    }  

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
        // RCLCPP_INFO(this->get_logger(), "PWM left: %d and PWM right:%d", pwm_left, pwm_right);
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param: params){
            if (param.get_name() == "sm_gain_ku"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_ku = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "sm_gain_kpsi"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_kpsi = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "sm_gain_kr"){
                if(param.as_double() >= 0.0 and param.as_double() < 100.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    sm_gain_kr = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "taud"){
                if(param.as_int() >= 0 and param.as_int() < 500){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    taud = param.as_int();
                    a=(taud*Ts)/(taud*Ts+Ts);
                    b=1/(taud*Ts+Ts);
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be between 0-100");
                    result.successful = false;
                    result.reason = "Value out of range";
                    return result;
                }
            }
            if (param.get_name() == "IGr_en"){
                if(param.as_double() == 0.0 or param.as_double() == 1.0){
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    IGr_en = param.as_double();
                }else{
                    RCLCPP_INFO(this->get_logger(), "could not change param value, should be 0.0 or 1.0");
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

    bool armed = false;
    float u_hat, psi_hat, r_hat, sig_u, sig_r, u_ref, psi_ref, r_ref, u_dot_ref, r_dot_ref;
    float c_ref;
    int count=0;
    //------Params-------//

    //------------------------------------------------------------------
    /*Parámetros del controlador MPC*/

    float Ts;                      // Tiempo de muestreo

    float W_u;                     // Peso para los errores de seguimiento de la referencia de surge
    float W_v;                     // Peso para los errores de seguimiento de la referencia de sway
    float W_psi;                   // Peso para los errores de seguimiento de la referencia de heading
    float W_r;                     // Peso para los errores de seguimiento de la referencia de yaw

    float W_R;                     // Peso para la acción de control del propulsor derecho 
    float W_L;                     // Peso para la acción de control del propulsor izquierdo 

    float lambda_terminalCost;     // Peso del coste terminal en la función objetivo

    int N_p;                       // Horizonte de predicción
    int N_c;                       // Horizonte de control
                                   
    float tau_f;                   // Constante de tiempo para el filtro del error de predicción

    MatrixXd K_d = MatrixXd::Zero(2, 6);
    MatrixXd P = MatrixXd::Zero(6, 6);
    
    float u_hat, psi_hat, r_hat, v_hat, sig_u, sig_v, sig_r, u_ref, psi_ref, r_ref;

    float T_R_ant, T_L_ant;

    float psi_est_ant, u_est_ant, v_est_ant, r_est_ant;

    float T_R_ant_est, T_L_ant_est;

    Sol_Delta_T_ant = VectorXd::Zero(n_u * N_c);
    factor_correccion_shifted = VectorXd::Zero(12); 
    factor_correccion_filtrado = VectorXd::Zero(6); 

    VectorXd Delta_T_max = (VectorXd(n_u) << Ts * Max_Force, Ts * Max_Force).finished();
    VectorXd Delta_T_min = -Delta_T_max;

    //------------------------------------------------------------------

    float sm_gain_ku; /*Ganancia del  controlador Sliding Modes (surge)*/
    float sm_gain_kpsi; /*Ganancia 1 del controlador Sliding Modes (yaw)*/
    float sm_gain_kr; /*Ganancia 2 del controlador Sliding Modes (yaw)*/
    
    float taud; /*Constante tau del filtro derivativo*/
    float a ,b; /*Constantes del filtro derivativo*/
    float IGr_en; /*Constantes del filtro derivativo*/

    float Xu6, Xu7, Xr11 , Xr13;  

    std::vector<float> memory_u; // Memoria para mantener los valores anteriores
    std::vector<float> memory_r; // Memoria para mantener los valores anteriores

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