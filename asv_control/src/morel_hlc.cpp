#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/state.hpp"                //Interface state mavros
#include "asv_interfaces/msg/state_observer.hpp"    //Interface state observer
#include "asv_interfaces/msg/state_neighbor.hpp"    //Interface state observer
// #include "asv_dev_library/HighLevelFlocking.hpp"  // todo create library
#include "asv_library/curvas_alamillo.h"

#include <cmath>
#include <thread>
#include <Eigen/Dense>

#include <queue>

using namespace Eigen;
using namespace std;

using std::placeholders::_1;

class Agent {
public:
    Matrix <float, 2,1> p; // Velocity
    Matrix <float, 2,1> q;
    float dt; // Time step

    // Constructor
    Agent(const Matrix<float, 2, 1>& pos, const Matrix<float, 2, 1>& vel, float muestreo)
        : p(vel), q(pos), dt(muestreo) {}

    // Update method
    void update(Matrix <float, 2, 1>& u) {
        q += dt * p;
        p += dt * u;
    }
};

// Helper functions

float sigma_norm(const Matrix<float, 2, 1>& x, float epsilon) {
    return (1.0 / epsilon) * (std::sqrt(1 + epsilon * x.squaredNorm()) - 1);
}

float sigma_norm(float x, float epsilon) {
    return (1.0 / epsilon) * (std::sqrt(1 + epsilon * x * x) - 1);
}

float sigma1(float x) {
    return x / std::sqrt(1 + x * x);
}

float ph(float z, float h) {
    if (z >= 0 && z < h) {
        return 1.0;
    } else if (z >= h && z <= 1.0) {
        return 0.5 * (1 + std::cos(M_PI * ((z - h) / (1 - h))));
    } else {
        return 0.0;
    }
}

float phi(float z, float a, float b) {
    float c = std::abs(a - b) / std::sqrt(4 * a * b);
    float p = z + c;
    float sigma_1 = p / std::sqrt(1 + p * p);
    return 0.5 * ((a + b) * sigma_1 + (a - b));
}

float phi_beta(float z, float d_beta, float h_beta) {
    float x = z / d_beta;
    float d = z - d_beta;
    float sigma1_d = sigma1(d);
    return ph(x, h_beta) * (sigma1_d - 1);
}

float phi_alpha(float z, float r_alpha, float d_alpha, float h, float a, float b) {
    float d = z - d_alpha;
    return ph(z / r_alpha, h) * phi(d, a, b);
}

// Matrix<float, 4, 1> particle_motion(float w) {
    
//     Target p_i = currentTarget(w);
//     xp_i=p_i.xp;
//     yp_i=p_i.yp;
//     dxp_i=p_i.dxp;
//     dyp_i=p_i.dyp;

//     Matrix<float, 4, 1> q_p_r = {0.4f*w + 20.0f*sin(0.02f*w), 0.4f*w, 0.4f + 0.4f*cos(0.02f*w), 0.4f};
//     return q_p_r;
// }

Matrix<float, 2, 1> alpha(const std::vector<Agent>& agents, float c1, float c2, float r, float r_alpha, int numParticles, int id, float epsilon, float h, float a, float b, float d_alpha) {
    Matrix <float, 2,1> suma1;
    Matrix <float, 2,1> suma2;
    suma1.setZero();
    suma2.setZero();
    
    Matrix <float, 2,1> q_dif, n_ij;

    for (int j = 0; j < numParticles; ++j) {
        if (j != id) {
            q_dif.setZero();
            q_dif = agents[j].q - agents[id].q;
            float norm_q_dif_s = q_dif.squaredNorm();

            if (norm_q_dif_s < r*r) {
                float z = sigma_norm(q_dif, epsilon);
                n_ij.setZero();
                n_ij = q_dif / std::sqrt(1 + epsilon * norm_q_dif_s);

                float va = z / r_alpha;
                float a_ij = ph(va, h);
                float u_alpha1_val = phi_alpha(z, r_alpha, d_alpha, h, a, b);
                Matrix <float, 2,1> u_alpha2;
                u_alpha2.setZero();

                u_alpha2 = a_ij * (agents[j].p - agents[id].p);
                suma1 += u_alpha1_val * n_ij;
                suma2 += u_alpha2;
            }
        }
    }

    Matrix<float, 2, 1> u_alpha = c1 * suma1 + c2 * suma2;
    return u_alpha;
}

Matrix<float, 2, 1> virtualLider(const std::vector<Agent>& agents, float c1, float c2, int id, const Matrix<float, 2, 1>& qr, const Matrix<float, 2, 1>& pr) {
    Matrix<float, 2, 1>  vv = agents[id].q - qr;
    vv /= std::sqrt(1 + vv.squaredNorm());

    Matrix<float, 2, 1> u_gamma = -c1 * vv - c2 * (agents[id].p - pr);
    
    return u_gamma;
}
/*
// std::vector<float> obstacles(const std::vector<Agent>& agents, const std::vector<std::vector<float>>& obs, int numObs, int id, float c1, float c2, float d, float h, float epsilon, float r_p, const std::vector<float>& Rk) {
//     std::vector<std::vector<float>> qiks;
//     std::vector<float> suma1(2, 0.0);
//     std::vector<float> suma2(2, 0.0);

//     for (int k = 0; k < numObs; ++k) {
//         std::vector<float> yk = obs[k];
//         std::vector<float> ydif(agents[id].q.size());
//         for (int i = 0; i < agents[id].q.size(); ++i) {
//             ydif[i] = agents[id].q[i] - yk[i];
//         }

//         float norm_ydif = 0.0;
//         for (float val : ydif) {
//             norm_ydif += val * val;
//         }
//         norm_ydif = std::sqrt(norm_ydif);

//         float mu = Rk[k] / norm_ydif;
//         std::vector<float> ak(ydif.size());
//         for (int i = 0; i < ydif.size(); ++i) {
//             ak[i] = ydif[i] / norm_ydif;
//         }

//         std::vector<std::vector<float>> P = {{1.0 - ak[0] * ak[0], -ak[0] * ak[1]},
//                                               {-ak[1] * ak[0], 1.0 - ak[1] * ak[1]}};

//         std::vector<float> qik(agents[id].q.size());
//         for (int i = 0; i < agents[id].q.size(); ++i) {
//             qik[i] = mu * agents[id].q[i] + (1 - mu) * yk[i];
//         }

//         std::vector<float> pik(agents[id].p.size());
//         for (int i = 0; i < agents[id].p.size(); ++i) {
//             pik[i] = mu * (P[i][0] * agents[id].p[0] + P[i][1] * agents[id].p[1]);
//         }

//         std::vector<float> qk_dif(qik.size());
//         for (int i = 0; i < qik.size(); ++i) {
//             qk_dif[i] = qik[i] - agents[id].q[i];
//         }

//         float norm_qk_dif = 0.0;
//         for (float val : qk_dif) {
//             norm_qk_dif += val * val;
//         }
//         norm_qk_dif = std::sqrt(norm_qk_dif);

//         if (norm_qk_dif < r_p) {
//             qiks.push_back(qik);
//             float qk_sigma = sigma_norm(qk_dif, epsilon);
//             std::vector<float> n_ik(qk_dif.size());
//             for (int i = 0; i < qk_dif.size(); ++i) {
//                 n_ik[i] = qk_dif[i] / std::sqrt(1 + epsilon * norm_qk_dif * norm_qk_dif);
//             }

//             float vb = qk_sigma / d;
//             float b_ik = ph(vb, h);

//             std::vector<float> u_beta1(qk_dif.size());
//             for (int i = 0; i < qk_dif.size(); ++i) {
//                 u_beta1[i] = phi_alpha(qk_sigma, d, h, 0.0, 1.0, 1.0) * n_ik[i];
//             }

//             std::vector<float> u_beta2(pik.size());
//             for (int i = 0; i < pik.size(); ++i) {
//                 u_beta2[i] = b_ik * (pik[i] - agents[id].p[i]);
//             }

//             for (int i = 0; i < suma1.size(); ++i) {
//                 suma1[i] += u_beta1[i];
//                 suma2[i] += u_beta2[i];
//             }
//         }
//     }

//     std::vector<float> u_beta(suma1.size());
//     for (int i = 0; i < suma1.size(); ++i) {
//         u_beta[i] = c1 * suma1[i] + c2 * suma2[i];
//     }

//     return u_beta;
// }
*/


class MorelHlcNode : public rclcpp::Node
{
public:
    MorelHlcNode() : Node("morel_hlc")
    {
        
        this-> declare_parameter("my_id", "ASV0");
        this-> declare_parameter("worker_mode", -1);
        this-> declare_parameter("Ts", 100.0);
        this-> declare_parameter("numParticles", 1);
        this-> declare_parameter("path_d", 0); // path_d = #Path deseado #


        my_string_id = (this->get_parameter("my_id").as_string());
        miid = std::stoi(my_string_id.substr(3));
        if (miid > 1)
            miid--;
        Ts = this->get_parameter("Ts").as_double()/1000.0;
        worker_mode = this->get_parameter("worker_mode").as_int();
        if (worker_mode == -1)
        {
            numParticles = 4;
        }
        else
        {
            numParticles = this->get_parameter("numParticles").as_int();
        }
        path_d  = this->get_parameter("path_d").as_int();
        
        RCLCPP_INFO(this->get_logger(),"Num Particles = %d",  numParticles);    
        X_s.resize(numParticles, -10.0);
        Y_s.resize(numParticles, -10.0);
        PSI_s.resize(numParticles, -10.0);
        u_s.resize(numParticles, -10.0);
        v_s.resize(numParticles, -10.0);

        this->declare_parameter("d", 20.0);
        this->declare_parameter("c1_alpha", 0.5);
        this->declare_parameter("c2_alpha", 0.5);
        this->declare_parameter("epsilon", 0.1);
        this->declare_parameter("h", 0.2);
        this->declare_parameter("a", 5.0);
        this->declare_parameter("b", 5.0);
        this->declare_parameter("c1_gamma", 3.0);
        this->declare_parameter("c2_gamma", 1.0);
        this->declare_parameter("max_sim_k", 500);
        this->declare_parameter("leader_delta_w", 0.003857);
        // Get parameter values
        d = this->get_parameter("d").as_double();
        c1_alpha = this->get_parameter("c1_alpha").as_double();
        c2_alpha = this->get_parameter("c2_alpha").as_double();
        epsilon = this->get_parameter("epsilon").as_double();
        
        r = d*1.2;
        r_alpha = sigma_norm(r, epsilon);
        d_alpha = sigma_norm(d, epsilon);

        RCLCPP_INFO(this->get_logger(), "r_alpha = %f", r_alpha);
        RCLCPP_INFO(this->get_logger(), "d_alpha = %f", d_alpha);
        
        h = this->get_parameter("h").as_double();
        // a = this->get_parameter("a").as_double();
        // b = this->get_parameter("b").as_double();

        c1_gamma = this->get_parameter("c1_gamma").as_double();
        c2_gamma = this->get_parameter("c2_gamma").as_double();
        max_sim_k = this->get_parameter("max_sim_k").as_int();
        delta_w = this->get_parameter("leader_delta_w").as_double();
        

        cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto options_sensors_ = rclcpp::SubscriptionOptions();
        options_sensors_.callback_group=cb_group_sensors_;

        publisher_mlc_ = this-> create_publisher<asv_interfaces::msg::StateObserver>("/" + my_string_id + "/control/output_hlc",
                rclcpp::SensorDataQoS());

        publisher_neighbor_mlc_ = this-> create_publisher<asv_interfaces::msg::StateNeighbor>("/" + my_string_id + "/neighbors/output_hlc",
                rclcpp::SensorDataQoS());
        
        subscriber_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateObserver>(
            "/" + my_string_id + "/observer/state_observer",rclcpp::SensorDataQoS(), std::bind(&MorelHlcNode::callbackStates,
            this, std::placeholders::_1), options_sensors_);

            
        subscriber_neighbor_states_obs_ = this-> create_subscription<asv_interfaces::msg::StateNeighbor>(
            "/" + my_string_id + "/neighbors/state_observer",rclcpp::SensorDataQoS(), std::bind(&MorelHlcNode::callbackNeighborStates,
            this, std::placeholders::_1), options_sensors_);

        subscriber_state = this-> create_subscription<mavros_msgs::msg::State>("/" + my_string_id + "/mavros/state",1,
                std::bind(&MorelHlcNode::callbackStateData, this, std::placeholders::_1), options_sensors_);

        timer_ = this -> create_wall_timer(std::chrono::milliseconds(int(Ts*1000.0)),
                std::bind(&MorelHlcNode::calculateHighLevelController, this), cb_group_obs_);

        RCLCPP_INFO(this->get_logger(), "\033[32mMorel HLC initialized\033[0m");
    }

private:
    void calculateHighLevelController()
    {
        if (armed == false)
        {
            if (!px_s.empty()){
                px_s = std::queue<float>();
                py_s = std::queue<float>();
                qx_s = std::queue<float>();
                qy_s = std::queue<float>();
            }
        }
        else{

            for(int i = 0; i < numParticles; ++i)
            {
                if (u_s[i] == -10.0)
                {
                    return;
                }
            }

            if (px_s.empty())
            {
                std::vector<float> X_k_s;
                std::vector<float> Y_k_s;
                std::vector<float> PSI_k_s;

                std::vector<float> vx_k_s;
                std::vector<float> vy_k_s;
                std::vector<float> u_k_s;
                std::vector<float> v_k_s;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    X_k_s = X_s;
                    Y_k_s = Y_s;
                    PSI_k_s = PSI_s;
                    u_k_s = u_s;
                    v_k_s = v_s;
                }


                std::vector<Agent> agents;
                Matrix <float, 3,3> ROT;
                Matrix <float, 3,1> global_vel_k;


                for (int i = 0; i < numParticles; ++i)
                {
                    global_vel_k.setZero();
                    ROT.setZero();

                    ROT << cos(PSI_k_s[i]) , -sin(PSI_k_s[i]) , 0,
                           sin(PSI_k_s[i]) , cos(PSI_k_s[i])  , 0,
                           0 , 0 , 1;
                    global_vel_k << u_k_s[i], v_k_s[i], 0;
                    global_vel_k = ROT * global_vel_k;

                    Matrix<float, 2, 1> initial_position = {X_k_s[i], Y_k_s[i]};
                    Matrix<float, 2, 1> initial_velocity = {global_vel_k(0,0), global_vel_k(1,0)};

                    agents.emplace_back(initial_position, initial_velocity, Ts);
                }


                // Test particle motion
                // TODO: cambiar a que w sea global float w = 0.0;
                Target p_i = currentTarget(w);
                Matrix<float, 2, 1> qr = {p_i.xp, p_i.yp};
                Matrix<float, 2, 1> pr = {p_i.dxp, p_i.dyp};

                for (int sim_k = 0; sim_k < max_sim_k; sim_k++)
                {
                    for (int i = 0; i < numParticles; ++i)
                    {

                        Matrix<float, 2, 1> u_alpha = alpha(agents, c1_alpha, c2_alpha, r, r_alpha, numParticles, i, epsilon, h, a, b, d_alpha);
                        Matrix<float, 2, 1> u_gamma = virtualLider(agents, c1_gamma, c2_gamma, i, qr, pr);
                        Matrix<float, 2, 1> control_input = {u_alpha[0] + u_gamma[0], u_alpha[1] + u_gamma[1]};

                        agents[i].update(control_input);
                    }
                    w += delta_w;
                    p_i = currentTarget(w);
                    qr = {p_i.xp, p_i.yp};
                    pr = {p_i.dxp, p_i.dyp};
                    if (worker_mode == -1){
                            qx_s.push(agents[miid].q[0]);
                            qy_s.push(agents[miid].q[1]);
                            px_s.push(agents[miid].p[0]);
                            py_s.push(agents[miid].p[1]);
                    }
                    else{
                        for (int i = 0; i < numParticles; ++i)
                        {

                            qx_s.push(agents[i].q[0]);
                            qy_s.push(agents[i].q[1]);
                            px_s.push(agents[i].p[0]);
                            py_s.push(agents[i].p[1]);
                        }
                    }
                }
                RCLCPP_INFO(this->get_logger(), "Finished Flocking, sending data");
            }


            if (worker_mode == -1){
                auto msg = asv_interfaces::msg::StateObserver();// point, velocity
                msg.header.stamp = this->now();
                msg.header.frame_id = my_string_id; 
                msg.point.x = qx_s.front();
                msg.point.y = qy_s.front();
                msg.velocity.x = px_s.front();
                msg.velocity.y = py_s.front();
                publisher_mlc_->publish(msg);

                qx_s.pop();
                qy_s.pop();
                px_s.pop();
                py_s.pop();
            }
            else{
                
                for (int i = 0; i < numParticles; ++i)
                {                
                    
                    if (i == 0)
                    {
                        auto msg = asv_interfaces::msg::StateObserver();// point, velocity
                        msg.header.stamp = this->now();
                        msg.header.frame_id = my_string_id; 
                        msg.point.x = qx_s.front();
                        msg.point.y = qy_s.front();
                        msg.velocity.x = px_s.front();
                        msg.velocity.y = py_s.front();
                        publisher_mlc_->publish(msg);
                    }
                    else
                    {
                        auto msg = asv_interfaces::msg::StateNeighbor();// point, velocity, id, msg_from
                        msg.id = std::to_string(i);
                        msg.msg_from = 0;
                        msg.point.x = qx_s.front();
                        msg.point.y = qy_s.front();
                        msg.velocity.x = px_s.front();
                        msg.velocity.y = py_s.front();
                        publisher_neighbor_mlc_->publish(msg);
                    }
                    qx_s.pop();
                    qy_s.pop();
                    px_s.pop();
                    py_s.pop();
                }
            }
        }
    }

    void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
    {
        armed= msg->armed;
    }

    void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg)
    {
        {
            if (worker_mode == -1)
            {
                std::lock_guard<std::mutex> lock(mutex_);
                X_s[miid] = msg->point.x; // X
                Y_s[miid] = msg->point.y; // Y
                PSI_s[miid] = msg->point.z; // psi
                u_s[miid] =  msg->velocity.x;
                v_s[miid] = msg->velocity.y;
            }
            else{
                std::lock_guard<std::mutex> lock(mutex_);
                X_s[0] = msg->point.x; // X
                Y_s[0] = msg->point.y; // Y
                PSI_s[0] = msg->point.z; // psi
                u_s[0] =  msg->velocity.x;
                v_s[0] = msg->velocity.y;
            }
        }
    }

    void callbackNeighborStates(const asv_interfaces::msg::StateNeighbor::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            int id = msg->msg_from; // esto no significa ID sino worker_mode
            if (id > 1)
                id--;
            X_s[id] = msg->point.x; // X
            Y_s[id] = msg->point.y; // Y
            PSI_s[id] = msg->point.z; // psi
            
            u_s[id] =  msg->velocity.x;
            v_s[id] = msg->velocity.y;
        }
    }

    Target currentTarget(float w){
        Target result;
        switch(path_d) {
            case 0:
                result.xp = w*Ts;
                result.yp = 0;
                result.dxp = delta_w;
                result.dyp = 0;
                break;
            case 1:
                // result = curva_sim_2_6(w);
                result = curva_ala_1_2(w);
                break;
            case 2:
                // result = curva_sim_2_8(w);
                result = curva_ala_1_4(w);
                break;
            case 3:
                // result = curva_sim_2_10(w);
                result = curva_ala_1_6(w);
                break;
            case 4:
                // result = curva_sim_3_6(w);
                result = curva_ala_2_2(w);
                break;
            case 5:
                // result = curva_sim_3_8(w);
                result = curva_ala_2_4(w);
                break;
            case 6:
                // result = curva_sim_3_10(w);
                result = curva_ala_2_6(w);
                break;
            case 7:
                result = curva_ala_3_1(w);
                break;
            case 8:
                result = curva_ala_3_2(w);
                break;
            case 9:
                result = curva_ala_3_3(w);
                break;
            case 10:
                result = curva_ala_4_2(w);
                break;
            case 11:
                result = curva_ala_4_3(w);
                break;
            case 12:
                result = curva_ala_4_4(w);
                break;
            case 13:
                result = curva_ala_5_4(w);
                break;
            case 14:
                result = curva_ala_5_5(w);
                break;
            case 15:
                result = curva_ala_5_6(w);
                break;
            case 16:
                result = curva_ala_6_2(w);
                break;
            case 17:
                result = curva_ala_6_3(w);
                break;
            case 18:
                result = curva_ala_6_4(w);
                break;
            case 19:
                result = curva_ala_7_4(w);
                break;
            case 20:
                result = curva_ala_7_5(w);
                break;
            case 21:
                result = curva_ala_7_6(w);
                break;
            case 22:
                result = curva_lissajous_1(w);
                break;
            case 23:
                result = curva_lissajous_2(w);
                break;
            case 24:
                result = curva_iav_1(w);
                break;
            default:
                result.xp =1.0;
                result.yp = 0.0;
                result.dxp = 1.0;
                result.dyp = 0.0;     
        }
        return result;
    }



    std::vector<float> X_s;
    std::vector<float> Y_s;
    std::vector<float> PSI_s;
    std::vector<float> u_s;
    std::vector<float> v_s;

    std::queue<float> px_s;
    std::queue<float> py_s;
    std::queue<float> qx_s;
    std::queue<float> qy_s;

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr subscriber_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateNeighbor>::SharedPtr subscriber_neighbor_states_obs_;
    rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_mlc_;
    rclcpp::Publisher<asv_interfaces::msg::StateNeighbor>::SharedPtr publisher_neighbor_mlc_;
    
    bool armed = false;
    std::string my_string_id; 
    // TODO: cambiar a parametros y luego agregar servicios para cambiarlos
    int miid= 0, numParticles = 0, worker_mode= 0;
    int max_sim_k = 500; // k = Ts
    float Ts;
    float delta_w = 0.0;
    float w = 0.0;
    float c1_alpha = 0.5;
    float c2_alpha = 0.5; 
    float d = 20.0;
    float r = 24.0;
    float epsilon = 0.1;
    float r_alpha = 66.55;
    float h = 0.2;
    float a = 5.0;
    float b = 5.0;
    float d_alpha = 54.03;
    float c1_gamma = 3.0;
    float c2_gamma = 1.0;
    int path_d; /*Variable para elegir path*/

// mutex callback group: 
    std::mutex mutex_;
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MorelHlcNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}


