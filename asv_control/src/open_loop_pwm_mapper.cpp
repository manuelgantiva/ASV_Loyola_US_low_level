#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/msg/rc_in.hpp"
#include "example_interfaces/srv/set_bool.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

using std::placeholders::_1;
using std::placeholders::_2;

class PwmMapperNode : public rclcpp::Node 
{
public:
    PwmMapperNode() : Node("open_loop_pwm_mapper"), current_time_(0.0)
    {
        // --- Declare and Get Parameters ---
        this->declare_parameter("my_id", "ASV0");
        this->declare_parameter("dt", 0.1);
        this->declare_parameter("phase_types", std::vector<int64_t>{});
        this->declare_parameter("phase_params1", std::vector<double>{});
        this->declare_parameter("phase_params2", std::vector<double>{});
        this->declare_parameter("phase_durations", std::vector<double>{});
        this->declare_parameter("interval_duration", 5.0);
        this->declare_parameter("fwd_dz", 1542.0);
        this->declare_parameter("rev_dz", 1466.0);
        
        my_id_ = this->get_parameter("my_id").as_string();
        dt_ = this->get_parameter("dt").as_double();
        phase_types_ = this->get_parameter("phase_types").as_integer_array();
        phase_params1_ = this->get_parameter("phase_params1").as_double_array();
        phase_params2_ = this->get_parameter("phase_params2").as_double_array();
        phase_durations_ = this->get_parameter("phase_durations").as_double_array();
        inter_dur_ = this->get_parameter("interval_duration").as_double();
        fwd_dz_ = this->get_parameter("fwd_dz").as_double();
        rev_dz_ = this->get_parameter("rev_dz").as_double();

        setup_sequence();

        publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("/" + my_id_ + "/mavros/rc/override", 10);
        server_ = this->create_service<example_interfaces::srv::SetBool>(
                "/" + my_id_ + "/control/on_off_pwm", std::bind(&PwmMapperNode::callbackOnOffPwm, this, _1, _2));
// Subscribed to RC_IN instead of /control/pwm_values
        subscriber_ = this->create_subscription<mavros_msgs::msg::RCIn>(
            "/" + my_id_ + "/mavros/rc/in", 10, std::bind(&PwmMapperNode::callbackRcIn, this, _1));

        RCLCPP_INFO(this->get_logger(), "Pwm Mapper loaded with %zu phases. Ready.", phase_types_.size());
    }

private:
    void setup_sequence() {
        double cumulative_time = 5.0; // Initial buffer
        phase_start_times_.push_back(cumulative_time);
        
        std::default_random_engine gen(std::random_device{}());
        std::uniform_real_distribution<double> d_mult(0.4, 1.0), d_freq(0.1, 1.2), d_phase(0.0, 2 * M_PI);

        for (size_t i = 0; i < phase_types_.size(); ++i) {
            cumulative_time += phase_durations_[i];
            phase_start_times_.push_back(cumulative_time);

            // Zig-Zag randoms
            std::vector<double> zz;
            for(int j=0; j < (int)(phase_durations_[i]/5.0 + 5); ++j) zz.push_back(d_mult(gen));
            zz_rand_mults_.push_back(zz);

            // Multi-Sine randoms
            std::vector<double> f, p;
            for(int j=0; j<4; ++j) { f.push_back(d_freq(gen)); p.push_back(d_phase(gen)); }
            ms_freqs_.push_back(f); ms_phases_.push_back(p);
        }
    }

    double applyDeadzone(double pwm) {
        if ((pwm > 1500.0 && pwm < fwd_dz_) || (pwm < 1500.0 && pwm > rev_dz_)) {
            return 1500.0;
        }
        return pwm;
    }

    void calculate_next_pwm(double &L, double &R) {
        auto it = std::lower_bound(phase_start_times_.begin(), phase_start_times_.end(), current_time_);
        size_t idx = std::distance(phase_start_times_.begin(), it);

        if (current_time_ < phase_start_times_[0] || idx > phase_types_.size()) {
            L = 1500; R = 1500;
        } else {
            int p_idx = idx - 1;
            int type = phase_types_[p_idx];
            double p1 = phase_params1_[p_idx];
            double p2 = phase_params2_[p_idx];
            double t_rel = current_time_ - phase_start_times_[p_idx];

            switch(type) {
                case 0: // Neutral
                    L = (t_rel < inter_dur_) ? smoothRamp(last_L_, 1500, inter_dur_, t_rel) : 1500;
                    R = (t_rel < inter_dur_) ? smoothRamp(last_R_, 1500, inter_dur_, t_rel) : 1500;
                    break;
                case 1: // Cruise
                    L = (t_rel < inter_dur_) ? smoothRamp(last_L_, p1, inter_dur_, t_rel) : p1;
                    R = (t_rel < inter_dur_) ? smoothRamp(last_R_, p1, inter_dur_, t_rel) : p1;
                    break;
                case 2: // J-Turn
                    L = (t_rel < inter_dur_) ? smoothRamp(last_L_, p1, inter_dur_, t_rel) : p1;
                    R = (t_rel < inter_dur_) ? smoothRamp(last_R_, p2, inter_dur_, t_rel) : p2;
                    break;
                case 3: // Zig-Zag
                    process_zigzag(p_idx, p1, p2, t_rel, L, R);
                    break;
                case 4: // Multi-Sine
                    process_multisine(p_idx, p1, p2, t_rel, L, R);
                    break;
            }
        }
        // Apply Deadzones before clamping
        L = applyDeadzone(L);
        R = applyDeadzone(R);
        L = std::clamp(L, 1100.0, 1900.0);
        R = std::clamp(R, 1100.0, 1900.0);
        last_L_ = L; last_R_ = R;
    }

    void process_zigzag(int p_idx, double p1, double p2, double t_rel, double &L, double &R) {
        if (t_rel < 5) {
            L = smoothRamp(last_L_, p1, 5, t_rel); R = smoothRamp(last_R_, p1, 5, t_rel);
        } else {
            double zz_t = t_rel - 5.0;
            int s_idx = std::min((int)(zz_t / 5.0), (int)zz_rand_mults_[p_idx].size() - 2);
            double blend = 0.5 - 0.5 * std::cos(M_PI * std::fmod(zz_t, 5.0) / 5.0);
            double amp_c = p2 * zz_rand_mults_[p_idx][s_idx];
            double amp_p = (s_idx == 0) ? 0 : p2 * zz_rand_mults_[p_idx][s_idx-1];
            
            double pL, pR, vL, vR;
            if (s_idx % 2 == 0) { pL=p1+amp_c; pR=p1-amp_c; vL=p1-amp_p; vR=p1+amp_p; }
            else { pL=p1-amp_c; pR=p1+amp_c; vL=p1+amp_p; vR=p1-amp_p; }
            if (s_idx == 0) { vL=p1; vR=p1; }
            L = vL + (pL - vL) * blend; R = vR + (pR - vR) * blend;
        }
    }

    void process_multisine(int p_idx, double p1, double p2, double t_rel, double &L, double &R) {
        auto& w = ms_freqs_[p_idx]; auto& phi = ms_phases_[p_idx];
        double wL = (p2*0.6)*std::sin(w[0]*t_rel + phi[0]) + (p2*0.4)*std::sin(w[1]*t_rel + phi[1]);
        double wR = (p2*0.5)*std::sin(w[2]*t_rel + phi[2]) + (p2*0.5)*std::sin(w[3]*t_rel + phi[3]);
        L = (t_rel < 5) ? smoothRamp(last_L_, p1+wL, 5, t_rel) : p1 + wL;
        R = (t_rel < 5) ? smoothRamp(last_R_, p1+wR, 5, t_rel) : p1 + wR;
    }

    double smoothRamp(double vs, double ve, double d, double tr) { return vs + (ve - vs) * std::clamp(tr/d, 0.0, 1.0); }

    void callbackRcIn(const mavros_msgs::msg::RCIn::SharedPtr msg)
{
        auto msg_override = mavros_msgs::msg::OverrideRCIn();
        msg_override.channels.fill(0); 

        double L, R;

        if (msg->channels[4] < fwd_dz_ || msg->channels[4] > rev_dz_) {
            // --- MODE: INTERNAL SEQUENCE ---
            calculate_next_pwm(L, R);
            current_time_ += dt_;
            L = applyDeadzone(L);
            R = applyDeadzone(R);
        } else {
            // --- MODE: MANUAL PASS-THROUGH (SKID STEERING MIXER) ---
            double steering_pwm = static_cast<double>(msg->channels[0]);
            double throttle_pwm = static_cast<double>(msg->channels[4]);

            // 1. Normalize inputs to a [-1.0, 1.0] range around the 1500 center
            double steering_norm = (steering_pwm - 1500.0) / 400.0;
            double throttle_norm = (throttle_pwm - 1500.0) / 400.0;

            // 2. Apply basic differential mixing
            double l_norm = throttle_norm + steering_norm;
            double r_norm = throttle_norm - steering_norm;

            // 3. Prevent saturation (ArduPilot prioritization)
            // If the combined commands ask for more than 100% motor output,
            // we scale them down equally to preserve the turn radius.
            double max_mag = std::max(std::abs(l_norm), std::abs(r_norm));
            if (max_mag > 1.0) {
                l_norm /= max_mag;
                r_norm /= max_mag;
            }

            // 4. Convert back to PWM space [1100, 1900]
            L = 1500.0 + (l_norm * 400.0);
            R = 1500.0 + (r_norm * 400.0);
        }

        msg_override.channels[9] = static_cast<uint16_t>(std::clamp(L, 1100.0, 1900.0));
        msg_override.channels[10] = static_cast<uint16_t>(std::clamp(R, 1100.0, 1900.0));

        publisher_->publish(msg_override);
    }

    void callbackOnOffPwm(const example_interfaces::srv::SetBool::Request::SharedPtr req, 
                         const example_interfaces::srv::SetBool::Response::SharedPtr res) {
        on_off_pwm = req->data;
        if(on_off_pwm) current_time_ = 0;
        res->success = true;
        res->message = "Sequence Status Updated";
    }

    std::string my_id_;
    double dt_, current_time_, last_L_ = 1500, last_R_ = 1500, inter_dur_;
    double fwd_dz_, rev_dz_;
    bool on_off_pwm = false;
    std::vector<int64_t> phase_types_;
    std::vector<double> phase_params1_, phase_params2_, phase_durations_, phase_start_times_;
    std::vector<std::vector<double>> zz_rand_mults_, ms_freqs_, ms_phases_;
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr publisher_;
    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr server_;
    rclcpp::Subscription<mavros_msgs::msg::RCIn>::SharedPtr subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PwmMapperNode>());
    rclcpp::shutdown();
    return 0;
}
