#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "geometry_msgs/msg/vector3.hpp"            //Interface reference_llc x->u y->r z->psi
#include "std_msgs/msg/float64.hpp"                 //Interface ref vel mid level controller
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/reference_llc.hpp"

#include <cmath>
#include <algorithm>
// #include "asv_library/curvas_sim.h"
#include "asv_library/curvas_alamillo.h"

using namespace std;

using std::placeholders::_1;

// Declaración de contantes

class IblosMlcNode : public rclcpp::Node
{
public:
    IblosMlcNode() : Node("iblos_mlc")
    {     
        std::string my_id; 
        this-> declare_parameter("my_id", "ASV0");
        
        //---------Parámetros del LLC-------------------//
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("delta_LOS", 8.0);
        this-> declare_parameter("k_u_tar", 2.0);
        this-> declare_parameter("k_b", 0.01);
        this-> declare_parameter("Xv_bar", std::vector<double>{0.7247758, -0.2352875, -0.0178921});
        this-> declare_parameter<double>("Eps", 0.2488);
        this-> declare_parameter("taud", 15.0); // Taud = #*Ts Est es #
        this-> declare_parameter("path_d", 0); // path_d = #Path deseado #
        this-> declare_parameter("u_max", 1.2); 
        this->declare_parameter<double>("r_ref_max", 0.6);
        this->declare_parameter<double>("r_ref_min", -0.6);
        this-> declare_parameter("SLOS_on", true); 
        this->declare_parameter("beta_bar_dot_max", 0.03);

        my_id = (this->get_parameter("my_id").as_string());    
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        delta_LOS = this->get_parameter("delta_LOS").as_double();
        k_u_tar = this->get_parameter("k_u_tar").as_double();
        k_b = this->get_parameter("k_b").as_double();
        Xv_bar = this->get_parameter("Xv_bar").as_double_array();
        this->get_parameter("Eps", Eps_);
        taud = this->get_parameter("taud").as_double();
        path_d  = this->get_parameter("path_d").as_int();
        u_max = this->get_parameter("u_max").as_double();
        this->get_parameter("r_ref_max", r_ref_max_);
        this->get_parameter("r_ref_min", r_ref_min_);
        bool SLOS_on = this->get_parameter("SLOS_on").as_bool();
        LOS = static_cast<double>(SLOS_on);
        beta_bar_dot_max = this->get_parameter("beta_bar_dot_max").as_double();

        memory_psi.fill(0.0f);

        msg_ref.references.resize(1);
        msg_ref.u_tar.data = 0.0;

        double denom = Ts * (taud + 1.0);
        a = (taud * Ts) / denom;
        b = 1.0 / denom;
        w = 0.0;

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->get_node_base_interface()->get_default_callback_group();
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        params_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&IblosMlcNode::param_callback, this, _1));

        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&IblosMlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);
        subscriber_references_ = this-> create_subscription<std_msgs::msg::Float64>(
            "/" + my_id + "/control/reference_mlc", 1, std::bind(&IblosMlcNode::callbackVelReference,
            this, std::placeholders::_1), options_sensors_);
        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_id + "/mavros/state",1,
                std::bind(&IblosMlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);
        publisher_llc = this-> create_publisher<asv_interfaces::msg::ReferenceLlc>("/" + my_id + "/control/reference_llc",1);
        publisher_error = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/error_mlc",1);
        publisher_los_state = this-> create_publisher<geometry_msgs::msg::Vector3>("/" + my_id + "/control/los_state_mlc",1);

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&IblosMlcNode::calculateMidLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "Mid Level Controller IGBLOS Node in %s has been started.", my_id.c_str());
    	
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
            memory_psi.fill(0.0f);
            armed_act = false;
            count=0;
            w=0.0;
            laps=0;
            beta_bar=0;
            v_bar_ff=0;
        }else{
            if(count > 4){
                //auto start = std::chrono::high_resolution_clock::now();
                auto msg_e = geometry_msgs::msg::Vector3();
                auto msg_los = geometry_msgs::msg::Vector3();

                double x_hat_i;
                double y_hat_i;
                double u_hat_i;
                double v_hat_i;
                double psi_hat_i;
                double u_d_i;
                double xe, ye;
                double xp_i, yp_i;
                double phip_i;
                double dphip_i;
                double Fp_i;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    x_hat_i = x_hat;
                    y_hat_i = y_hat;
                    u_hat_i = u_hat;
                    v_hat_i = v_hat;
                    psi_hat_i=psi_hat;
                    u_d_i=u_d;
                }
                Target p_i = currentTarget(w);
                xp_i=p_i.xp;
                yp_i=p_i.yp;
                phip_i = p_i.phip;
                dphip_i = p_i.dphip;
                Fp_i = p_i.f_c;

                psi_hat_i = normalizeAngle(psi_hat_i);

                double beta_hat_i = atan2(v_hat_i,u_hat_i);
                double chi_hat =  psi_hat_i + beta_hat_i;

                xe = (x_hat_i - xp_i)*cos(phip_i) + (y_hat_i - yp_i)*sin(phip_i);
                ye = -1*(x_hat_i - xp_i)*sin(phip_i) + (y_hat_i - yp_i)*cos(phip_i);

                msg_e.x = xe;
                msg_e.y = ye;
                msg_e.z = w;

                double k1_i = u_d_i / delta_LOS;
                double u_ref = k1_i * std::sqrt(delta_LOS*delta_LOS + ye*ye*LOS);
                double U_hat = std::sqrt(u_hat_i*u_hat_i + v_hat_i*v_hat_i);
                double u_tar = k_u_tar*xe + U_hat*cos(chi_hat-phip_i);
                u_tar = std::clamp(u_tar, 0.0, 3.0);
                double w_dot = u_tar / Fp_i;

                double chi_d = phip_i - std::atan2(ye, delta_LOS);

                const double chi_e = std::atan2(std::sin(chi_hat - chi_d), std::cos(chi_hat - chi_d));
                double beta_bar_dot = k_b * chi_e;
                beta_bar_dot = std::clamp(beta_bar_dot, -beta_bar_dot_max, beta_bar_dot_max);
                beta_bar += beta_bar_dot * Ts;
                // RCLCPP_INFO(this->get_logger(), "dot: %.5f and beta %.5f", beta_bar_dot, beta_bar);

                const double chi_d_dot = dphip_i * u_tar / Fp_i;
                const double dot_v_bar_ff = Xv_bar[0]*u_d_i * chi_d_dot + Xv_bar[1] * v_bar_ff + (Xv_bar[2]-Eps_*Xv_bar[1])*chi_d_dot;
                v_bar_ff += dot_v_bar_ff * Ts;
                const double v_ff = v_bar_ff - Eps_*chi_d_dot;
                const double beta_ff = std::atan2(v_ff, u_d_i);

                const double beta_est = beta_bar + beta_ff;
                double psi_ref = chi_d - beta_est;

                msg_los.x = beta_bar;
                msg_los.y = beta_ff;
                msg_los.z = beta_bar_dot;

                w += Ts*w_dot;
                // Corrijo el angulo de referencia teniendo en cuenta las vueltas sobre la trayectoria
                if(psi_ref<0){
                    psi_ref=psi_ref+(2*M_PI);
                }

                if(armed_act==false){
                    psi_ant = psi_ref;
                    laps = 0;
                }else{
                    if((psi_ref - psi_ant) > M_PI){
                        laps = laps - 1;
                    }else if((psi_ref - psi_ant) < -M_PI){
                        laps = laps + 1;
                    }
                    psi_ant=psi_ref;
                    psi_ref += 2*M_PI*laps;
                }
                        
                double r_ref = derivationFilter(psi_ref, memory_psi, a, b);

                double r_ref_max = 0.6;
                if(r_ref > r_ref_max){
                    r_ref = r_ref_max;
                }else if(r_ref < -r_ref_max){
                    r_ref = -r_ref_max;
                }

                if(u_ref > u_max){
                    u_ref = u_max;
                }

                msg_ref.references[0].x = u_ref;
                msg_ref.references[0].y = r_ref;
                msg_ref.references[0].z = psi_ref;
                msg_ref.u_tar.data = u_tar;
                publisher_llc->publish(msg_ref);
                publisher_error->publish(msg_e);
                publisher_los_state->publish(msg_los);
                armed_act = true;
                // auto end = std::chrono::high_resolution_clock::now();
                // std::chrono::duration<double> elapsed = end - start;
                // double miliseconds = elapsed.count()*1000;
                // Imprime el tiempo con dos decimales fijos
                // RCLCPP_INFO(this->get_logger(), "Exec time: %.2f milliseconds", miliseconds);
            }else{
                msg_ref.references[0].x = 0.5;
                msg_ref.references[0].y = 0.0;
                msg_ref.references[0].z = 0.0;
                msg_ref.u_tar.data = 0.0;
                count=count+1;
                publisher_llc->publish(msg_ref);
            }
        }       
    }

    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            x_hat = msg->point.x;
            y_hat = msg->point.y;
            u_hat = msg->velocity.x;
            v_hat = msg->velocity.y;
            psi_hat = msg->point.z;
        }
    }

    void callbackVelReference(const std_msgs::msg::Float64::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            u_d = msg->data;
        }
    }

    // Función para calcular la salida del filtro de derivación
    double derivationFilter(double input, std::array<double,2>& memory, double a, double b){
        const double previous_output = memory[0];
        const double previous_input  = memory[1];

        double output = (a * previous_output) + b * (input - previous_input);

        memory[0] = output;
        memory[1] = input;

        return output;
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            armed= msg->armed;
        }
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
        

    double normalizeAngle(double angle)    {
        const double twoPi = 2.0 * M_PI;
        angle = std::fmod(angle, twoPi);
        if (angle < 0.0)
            angle += twoPi;
        return angle;
    }

    rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> &params){
        rcl_interfaces::msg::SetParametersResult result;
        bool armed_local;

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
            if (param.get_name() == "u_max") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() <= 2.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    u_max = param.as_double();
                } else {
                    return reject("could not change param u_max",
                                "u_max: double in [0,2.0]");
                }
            }
            if (param.get_name() == "taud") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() >= 0.0 && param.as_double() < 500.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    taud = param.as_double();
                    a = (taud * Ts) / (taud * Ts + Ts);
                    b = 1.0 / (taud * Ts + Ts);
                } else {
                    return reject("could not change param taud",
                                "taud: double in [0,500)");
                }
            }
            if (param.get_name() == "path_d") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
                    param.as_int() >= 0 && param.as_int() <= 5) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    path_d = param.as_int();
                } else {
                    return reject("could not change param path_d",
                                "path_d: integer in [0,5]");
                }
            }
            if (param.get_name() == "Ts") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
                    param.as_double() > 0.0 && param.as_double() < 1000.0) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    Ts = param.as_double() / 1000.0;

                    a = (taud * Ts) / (taud * Ts + Ts);
                    b = 1.0 / (taud * Ts + Ts);

                    if (timer_) {
                        timer_->cancel();
                    }

                    timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(int(Ts * 1000.0)),
                        std::bind(&IblosMlcNode::calculateMidLevelController, this),
                        cb_group_obs_);
                } else {
                    return reject("could not change param Ts",
                                "Ts: double in (0,1000) ms");
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
            if (param.get_name() == "SLOS_on") {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                    RCLCPP_INFO(this->get_logger(), "changed param value");
                    bool SLOS_on = param.as_bool();
                    LOS = static_cast<double>(SLOS_on);
                } else {
                    return reject("could not change param SLOS_on",
                                "SLOS_on: bool");
                }
            }
        }
        result.successful = true;
        result.reason = "Success";
        return result;
    }

    bool armed = false, armed_act=false;
    double u_hat = 0, psi_hat = 0, r_hat = 0, v_hat = 0, x_hat = 0, y_hat = 0, u_d = 0, w=0.0, psi_ant;
    int count=0, laps=0;
    double  beta_bar=0, v_bar_ff=0, Eps_;
    std::vector<double> Xv_bar;
    //------Params-------//
    float Ts;  
    /*Parámetros del controlador LOS*/
    double delta_LOS, LOS; /*Ganancia delta SGLOS*/
    double k_u_tar; /*Ganancia de la velocidad de surge target*/
    double k_b; /*Ganancia de la velocidad de surge target*/
    double u_max; /*velocidad maxima de referencia*/
    
    double taud; /*Constante tau del filtro derivativo*/
    double a ,b; /*Constantes del filtro derivativo*/

    int path_d; /*Variable para elegir path*/
    double r_ref_max_, r_ref_min_;
    double beta_bar_dot_max;

    std::array<double, 2> memory_psi{};
    asv_interfaces::msg::ReferenceLlc msg_ref;

    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscriber_references_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Publisher<asv_interfaces::msg::ReferenceLlc>::SharedPtr publisher_llc;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_error;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_los_state;
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
    auto node = std::make_shared<IblosMlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}