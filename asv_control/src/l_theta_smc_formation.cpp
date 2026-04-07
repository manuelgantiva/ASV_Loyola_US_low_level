    #include "rclcpp/rclcpp.hpp"
    #include <cmath>
    #include <mutex>
    #include <Eigen/Dense>
    #include "asv_interfaces/msg/state_observer.hpp"
    #include "asv_interfaces/msg/state_neighbor.hpp"
    #include "asv_interfaces/msg/pwm_values.hpp"
    #include "mavros_msgs/msg/state.hpp"
    #include "geometry_msgs/msg/vector3.hpp"
    #include "std_msgs/msg/float64_multi_array.hpp"
    #include "std_msgs/msg/string.hpp"

    using namespace Eigen;
    using namespace std::chrono_literals;

    class LThetaSMCFormation : public rclcpp::Node
    {
    public:
        LThetaSMCFormation() : Node("l_theta_smc_formation")
        {
            //std::string my_id; 
            // ******************** Parameter Declaration ********************
            this->declare_parameter("my_id", "ASV0");
            this->declare_parameter("Ts", 100.0);
            this->declare_parameter("M11", 85.2515);
            this->declare_parameter("M22", 162.5000);
            this->declare_parameter("M33", 41.4451);
            this->declare_parameter("D11", -77.5541);
            this->declare_parameter("D22", -162.5000);
            this->declare_parameter("D33", -41.4451);
            this->declare_parameter("L12_d",6.0);
            this->declare_parameter("Theta12_d", M_PI/2);
            this->declare_parameter("d_vessel", 0.5);
            this->declare_parameter("phi1", 0.05);
            this->declare_parameter("phi2", 0.05);
            this->declare_parameter("K1", 0.9);
            this->declare_parameter("K2", 0.9);
            this->declare_parameter("Lambda1", 0.8);
            this->declare_parameter("Lambda2", 0.8);
            
            // Polynomial coefficients for thrust mapping
            this->declare_parameter("mf0", 0.0013545);
            this->declare_parameter("mf1", 6.0977);
            this->declare_parameter("mf2", 0.0);
            this->declare_parameter("mf3", -2.769);
            this->declare_parameter("mf4", 0.0);
            this->declare_parameter("mf5", -1.0978);

            this->declare_parameter("mr0", -0.0059858);
            this->declare_parameter("mr1", 6.1789);
            this->declare_parameter("mr2", 0.20095);
            this->declare_parameter("mr3", -5.1266);
            this->declare_parameter("mr4", 1.048);
            this->declare_parameter("mr5", -2.5286);

            this->declare_parameter("df0", 0.0);
            this->declare_parameter("df1", 0.0);
            this->declare_parameter("df2", 6.3681);
            this->declare_parameter("df3", 0.0);
            this->declare_parameter("df4", 8.2298);
            this->declare_parameter("df5", 0.0);

            this->declare_parameter("dr0", 0.030548);
            this->declare_parameter("dr1", -2.8142);
            this->declare_parameter("dr2", 5.3685);
            this->declare_parameter("dr3", 27.237);
            this->declare_parameter("dr4", 4.2689);
            this->declare_parameter("dr5", 13.881);
            
            //Input gain limits
            this->declare_parameter("IGumax_ff", 0.17794);  
            this->declare_parameter("IGumax_rf", 0.08897);  
            this->declare_parameter("IGumin_rf", -0.07331); 
            this->declare_parameter("IGrmax_ff", 0.14128);  
            this->declare_parameter("IGrmax_rf", 0.22898); 

            // Deadzone compensation
            this->declare_parameter("Dz_up", 0.0750);    // Positive deadzone offset
            this->declare_parameter("Dz_down", -0.08);   // Negative deadzone offset
            
            my_id = this->get_parameter("my_id").as_string();
            Ts_   = this->get_parameter("Ts").as_double();
            M11_  = this->get_parameter("M11").as_double();
            M22_  = this->get_parameter("M22").as_double();
            M33_  = this->get_parameter("M33").as_double();

            if (M22_ == 0.0) {
                RCLCPP_FATAL(get_logger(), "M22 cannot be zero!");
                rclcpp::shutdown();
            }

            mr_ = M11_ / M22_;
            md_ = M22_ - M11_;

            D11_ = this->get_parameter("D11").as_double();
            D22_ = this->get_parameter("D22").as_double();
            D33_ = this->get_parameter("D33").as_double();

            L12_d_ = this->get_parameter("L12_d").as_double();
            Theta12_d_ = this->get_parameter("Theta12_d").as_double();
            d_vessel_ = this->get_parameter("d_vessel").as_double();

            phi1_ = this->get_parameter("phi1").as_double();
            phi2_ = this->get_parameter("phi2").as_double();
            K1_ = this->get_parameter("K1").as_double();
            K2_ = this->get_parameter("K2").as_double();
            Lambda1_ = this->get_parameter("Lambda1").as_double();
            Lambda2_ = this->get_parameter("Lambda2").as_double();
            // Polynomial coefficients for thrust mapping
            mf0 = this->get_parameter("mf0").as_double();
            mf1 = this->get_parameter("mf1").as_double();
            mf2 = this->get_parameter("mf2").as_double();
            mf3 = this->get_parameter("mf3").as_double();
            mf4 = this->get_parameter("mf4").as_double();
            mf5 = this->get_parameter("mf5").as_double();
            mr0 = this->get_parameter("mr0").as_double();
            mr1 = this->get_parameter("mr1").as_double();
            mr2 = this->get_parameter("mr2").as_double();
            mr3 = this->get_parameter("mr3").as_double();
            mr4 = this->get_parameter("mr4").as_double();
            mr5 = this->get_parameter("mr5").as_double();
            df0 = this->get_parameter("df0").as_double();
            df1 = this->get_parameter("df1").as_double();
            df2 = this->get_parameter("df2").as_double();
            df3 = this->get_parameter("df3").as_double();
            df4 = this->get_parameter("df4").as_double();
            df5 = this->get_parameter("df5").as_double();
            dr0 = this->get_parameter("dr0").as_double();
            dr1 = this->get_parameter("dr1").as_double();
            dr2 = this->get_parameter("dr2").as_double();
            dr3 = this->get_parameter("dr3").as_double();
            dr4 = this->get_parameter("dr4").as_double();
            dr5 = this->get_parameter("dr5").as_double();

            IGumax_ff = this->get_parameter("IGumax_ff").as_double();
            IGumax_rf = this->get_parameter("IGumax_rf").as_double();
            IGumin_rf = this->get_parameter("IGumin_rf").as_double();
            IGrmax_ff = this->get_parameter("IGrmax_ff").as_double();
            IGrmax_rf = this->get_parameter("IGrmax_rf").as_double();

            Dz_1= this->get_parameter("Dz_up").as_double();
            Dz_2 = this->get_parameter("Dz_down").as_double();

            this->declare_parameter("Z_d", std::vector<double>{Theta12_d_, L12_d_});
            
            cb_group_sensors_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            cb_group_obs_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            auto options_sensors_ = rclcpp::SubscriptionOptions();
            options_sensors_.callback_group = cb_group_sensors_;

            setup_comms(options_sensors_);
            RCLCPP_INFO(get_logger(), "L-Theta SMC Formation Controller initialized");
            
        }

    private:
        rclcpp::QoS qos{rclcpp::QoS(10).reliable()};
        std::string my_id;
        struct State {
            double x, y, psi;
            double u, v, r;
            double sigma_u, sigma_v, sigma_r;
        };


        void setup_comms(const rclcpp::SubscriptionOptions& options_sensors_) 
        {
             
            arm_disarm_state = create_subscription<mavros_msgs::msg::State>(
                "/" + my_id + "/mavros/state", 1,
                std::bind(&LThetaSMCFormation::callbackStateData, this, std::placeholders::_1), 
                options_sensors_);

            // Subscriber to the state of the This vessel
            this_vessel_states_obs_ = create_subscription<asv_interfaces::msg::StateObserver>(
                "/" + my_id + "/observer/state_observer",
                rclcpp::SensorDataQoS(),
                std::bind(&LThetaSMCFormation::callbackStates, this, std::placeholders::_1),
                options_sensors_);


            // Subscriber to the state of the neighbor
            
            //subscriber_state_neighbor_ = this->create_subscription<asv_interfaces::msg::StateNeighbor>(
            //    "/" + std::to_string(my_id_) + "/neighbors/state_neighbor",
            //    rclcpp::SensorDataQoS(),
            //    std::bind(&LThetaSMCFormation::callbackNeighbor, this, std::placeholders::_1),
            //options_sensors_);

            subscriber_state_neighbor_ = this->create_subscription<asv_interfaces::msg::StateNeighbor>(
            "/" + my_id + "/neighbors/state_observer",rclcpp::SensorDataQoS(),
            std::bind(&LThetaSMCFormation::callbackNeighbor, this, std::placeholders::_1),
            options_sensors_);

            // Publishers for this controller
            control_pub_   = create_publisher<geometry_msgs::msg::Vector3>("control_commands", rclcpp::SensorDataQoS());
            formation_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("formation_values", rclcpp::SensorDataQoS());
            error_pub_     = create_publisher<std_msgs::msg::Float64MultiArray>("l_theta_error", rclcpp::SensorDataQoS());
            slider_pub_    = create_publisher<std_msgs::msg::Float64MultiArray>("sliding_surfaces", rclcpp::SensorDataQoS());
            wave_pub_      = create_publisher<std_msgs::msg::Float64MultiArray>("wave_disturbances", rclcpp::SensorDataQoS());
            W_effect_pub_  = create_publisher<std_msgs::msg::Float64MultiArray>("w_effect", rclcpp::SensorDataQoS());

            // Publisher for the input gains
            pub_pwm_       = create_publisher<asv_interfaces::msg::PwmValues>("/" + get_parameter("my_id").as_string() + "/control/pwm_value_ifac",10);
            pub_IG_        = create_publisher<geometry_msgs::msg::Vector3>("IG_LTheta", rclcpp::SensorDataQoS());       
            // Optional test publisher
            // test_pub_ = create_publisher<std_msgs::msg::String>("test_topic", qos);

            // Control timer
            control_timer_ = create_wall_timer(
                std::chrono::milliseconds(static_cast<int64_t>(Ts_)),
                [this]() { control_update(); });
        }

        void callbackStates(const asv_interfaces::msg::StateObserver::SharedPtr msg) 
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
                follower_state_ = 
                {
                    msg->point.x, msg->point.y, msg->point.z,
                    msg->velocity.x, msg->velocity.y, msg->velocity.z,
                    msg->disturbances.x, msg->disturbances.y, msg->disturbances.z
                };
                follower_received_ = true;
        }

        void callbackNeighbor(const asv_interfaces::msg::StateNeighbor::SharedPtr msg) 
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
                leader_state_ =
                {
                    msg->point.x, msg->point.y, msg->point.z,
                    msg->velocity.x, msg->velocity.y, msg->velocity.z,
                    0.0, 0.0, 0.0  // Disturbances not used here
                };
            leader_received_ = true;
        }

        void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
        {
            prev_armed = armed;
            armed = msg->armed;

            // Reset counter on disarm
            if (!armed && prev_armed) { arm_counter = 0;}
        }

    void control_update() 
    {
        // Wait for arming and 7 iterations after arming
        if (!armed) 
        {
            arm_counter = 0;
            auto pwm_msg = asv_interfaces::msg::PwmValues();
            pwm_msg.t_left = 1500;
            pwm_msg.t_righ = 1500;
            pub_pwm_->publish(pwm_msg);
            return;
            
        }

        // If just armed, start counting
        if (arm_counter < ARM_WAIT_ITER) 
        {
            arm_counter++;
            auto pwm_msg = asv_interfaces::msg::PwmValues();
            pwm_msg.t_left = 1500;
            pwm_msg.t_righ = 1500;
            pub_pwm_->publish(pwm_msg);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "Waiting %d/%d cycles after arming before enabling controller...", 
                arm_counter, ARM_WAIT_ITER);
            return;
        }

        //else {
            
            //auto msg = asv_interfaces::msg::PwmValues();
            //auto msg_Igu = geometry_msgs::msg::Vector3();
            //auto msg_Igr = geometry_msgs::msg::Vector3();
            //auto msg_Ig = geometry_msgs::msg::Vector3();
            rclcpp::Time now = this->now();
            const rclcpp::Duration freshness_limit = rclcpp::Duration::from_seconds(0.5);

            State leader, follower;
            {
                std::lock_guard<std::mutex> lock(data_mutex_);

                if (!leader_received_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Leader state not received yet.");
                }
                if (!follower_received_) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                    "Follower state not received yet.");
                }
                if (!leader_received_ || !follower_received_) {
                    return;
                }

                leader = leader_state_;
                follower = follower_state_;
            }

            try 
            {
                //auto msg = asv_interfaces::msg::PwmValues();
                //auto msg_Igu = geometry_msgs::msg::Vector3();
                //auto msg_Igr = geometry_msgs::msg::Vector3();
                auto msg_Ig = geometry_msgs::msg::Vector3();

                RCLCPP_DEBUG(get_logger(), "Starting control cycle");
                std::vector<double> Z = compute_formation_params(leader, follower);
                publish_Z(Z);

                std::vector<double> Z_dot = compute_derivatives(leader, follower, Z);

                std::vector<double> Z_d = this->get_parameter("Z_d").as_double_array();
                std::vector<double> Z_tildas = {
                    Z[0] - Z_d[0],
                    Z[1] - Z_d[1]
                };
                publish_errors(Z_tildas);

                Matrix<double, 2, 3> S_matrix = compute_sliding_surfaces(Z_dot, Z_tildas);
                publish_sliding_surfaces(S_matrix);

                std::vector<double> wave_disturbances = get_wave_disturbances(follower);
                publish_wave(wave_disturbances);

                std::vector<double> W = compute_disturbance_effects(wave_disturbances, Z);
                publish_W(W);

                std::vector<double> fl_fthetha = compute_nonlinear_terms(follower, Z, Z_dot);

                std::vector<double> control_inputs = compute_control_inputs(fl_fthetha, Z, S_matrix, W);

                publish_control(control_inputs);
                
                double g_u_dot = 1.0 / M11_;
                double IG_u = g_u_dot * control_inputs[1];  // Apply input gain to surge control
                double g_r_dot = 1.0/(M33_ ); // It is M22_/(M22 * M33_ - M23_^2) But my controller is assuming that the mass Matrix is diagonal so it will be simplified to 1/m33
                double IG_r = g_r_dot * control_inputs[0];   // Apply input gain to Yaw control         
                
                if(IG_u > IGumax_ff) { 
                    IG_u = IGumax_ff;
                }
                if(IG_u < IGumin_rf) {
                    IG_u = IGumin_rf;
                }
                if(IG_r > IGrmax_rf) {
                    IG_r = IGrmax_rf;
                }
                if(IG_r < -IGrmax_rf) {
                    IG_r = -IGrmax_rf;
                }
                // Calculate the PWM limits
                // Calculate zone-based control allocation polynomials for thrust mapping
                float m, d_, zone;
                if(IG_u > IGumax_rf) {
                    // Red Zone: High surge input gain region
                    // Use forward-optimized polynomial coefficients
                    m = mf0 + mf1*IG_u + mf2*IG_r + mf3*IG_u*IG_u + mf4*IG_u*IG_r + mf5*IG_r*IG_r;
                    d_ = df0 + df1*IG_u + df2*IG_r + df3*IG_u*IG_u + df4*IG_u*IG_r + df5*IG_r*IG_r;
                    zone = 0;
                } else if(IG_r > IGrmax_ff) {
                    // Blue Zone: High positive yaw rate region
                    // Use rotation-optimized polynomial coefficients
                    m = mr0 + mr1*IG_u + mr2*IG_r + mr3*IG_u*IG_u + mr4*IG_u*IG_r + mr5*IG_r*IG_r;
                    d_ = dr0 + dr1*IG_u + dr2*IG_r + dr3*IG_u*IG_u + dr4*IG_u*IG_r + dr5*IG_r*IG_r;
                    zone = 1;
                } else if(IG_r < -IGrmax_ff) {
                    // Green Zone: High negative yaw rate region
                    // Same coefficients as Blue Zone but with sign adjustments for negative rotation
                    m = mr0 + mr1*IG_u - mr2*IG_r + mr3*IG_u*IG_u - mr4*IG_u*IG_r + mr5*IG_r*IG_r;
                    d_ = dr0 - dr1*IG_u + dr2*IG_r - dr3*IG_u*IG_u + dr4*IG_u*IG_r - dr5*IG_r*IG_r;
                    zone = -1;
                } else {
                    // Default to Red Zone with possible reclassification
                    // Start with forward motion coefficients
                    m = mf0 + mf1*IG_u + mf2*IG_r + mf3*IG_u*IG_u + mf4*IG_u*IG_r + mf5*IG_r*IG_r;
                    d_ = df0 + df1*IG_u + df2*IG_r + df3*IG_u*IG_u + df4*IG_u*IG_r + df5*IG_r*IG_r;
                    zone = 0;
                    // Check if thrust allocation is valid in this zone
                    if((m <= 0.5*d_) || (m <= -0.5*d_)) {
                        // Thruster saturation condition detected - reclassify
                        if(IG_r >= 0) {
                            // Blue Zone: Better for positive rotation
                            m = mr0 + mr1*IG_u + mr2*IG_r + mr3*IG_u*IG_u + mr4*IG_u*IG_r + mr5*IG_r*IG_r;
                            d_ = dr0 + dr1*IG_u + dr2*IG_r + dr3*IG_u*IG_u + dr4*IG_u*IG_r + dr5*IG_r*IG_r;
                            zone = 1;
                        } else {
                            // Green Zone: Better for negative rotation
                            m = mr0 + mr1*IG_u - mr2*IG_r + mr3*IG_u*IG_u - mr4*IG_u*IG_r + mr5*IG_r*IG_r;
                            d_ = dr0 - dr1*IG_u + dr2*IG_r - dr3*IG_u*IG_u + dr4*IG_u*IG_r - dr5*IG_r*IG_r;
                            zone = -1;
                        }
                    }
                }
                                // Update zone indicator in visualization message
                //msg_Ig = zone;
                //publisher_IG_->publish(msg_Ig);

                // Calculate left and right thruster commands using mean and differential thrust
                double L = ((2 * m + d_) / 2);
                double R = ((2 * m - d_) / 2);

                // Apply deadzone compensation
                if (L > 0) L += Dz_1;
                else if (L < 0) L += Dz_2;
                if (R > 0) R += Dz_1;
                else if (R < 0) R += Dz_1;

                // ******** ADD THIS BLOCK HERE ********
                auto pwm_msg = asv_interfaces::msg::PwmValues();
                msg_Ig.x = IG_u;
                msg_Ig.y = IG_r;
                msg_Ig.z = zone;

                pwm_msg.t_left = static_cast<uint16_t>(400 * L + 1500);
                pwm_msg.t_righ = static_cast<uint16_t>(400 * R + 1500);

                
                pwm_msg.t_left = std::clamp(pwm_msg.t_left, static_cast<uint16_t>(1100), static_cast<uint16_t>(1900));
                pwm_msg.t_righ = std::clamp(pwm_msg.t_righ, static_cast<uint16_t>(1100), static_cast<uint16_t>(1900));

                
                pub_pwm_->publish(pwm_msg);
                pub_IG_->publish(msg_Ig);
                RCLCPP_INFO(get_logger(), "Published input gains: IG_u=%.3f, IG_r=%.3f, zone=%d",
                            msg_Ig.x, msg_Ig.y, static_cast<int>(msg_Ig.z));
                RCLCPP_INFO(get_logger(), "Published PWM MESSAEGES: pwm_right_ltheta=%.3d, pwm_left_ltheta=%.3d",
                            pwm_msg.t_righ, pwm_msg.t_left);
                // ******** END OF ADDED BLOCK ********
            
            }
            
            catch(const std::exception& e) {RCLCPP_ERROR(get_logger(), "Control error: %s", e.what());}
        //}
        
    }

    std::vector<double> compute_formation_params(const State& leader, const State& follower) {
        const double xp2 = follower.x + d_vessel_ * cos(follower.psi);
        const double yp2 = follower.y + d_vessel_ * sin(follower.psi);

        const double dx = xp2 - leader.x;
        const double dy = yp2 - leader.y;

        Matrix3d R = rotation_matrix(leader.psi);
        Vector3d relative_pos = R.transpose() * Vector3d(dx, dy, 0);

        const double l12 = sqrt((dx * dx) +  (dy*dy));
        const double theta_12 = atan2(relative_pos[1], relative_pos[0]);
        const double alpha_zero = theta_12 + leader.psi;
        const double gama1 = alpha_zero - follower.psi;

        return {theta_12, l12, alpha_zero, gama1};
    }

    std::vector<double> compute_derivatives(const State& leader, const State& follower,
                                           const std::vector<double>& Z) {
        Matrix3d R_psil = rotation_matrix(leader.psi);
        Vector3d eta_dotl = R_psil * Vector3d(leader.u, leader.v, leader.r);

        Matrix3d R_psif = rotation_matrix(follower.psi);
        Vector3d eta_dotf = R_psif * Vector3d(follower.u, follower.v, follower.r);

        const double x1_dot = eta_dotl[0], y1_dot = eta_dotl[1];
        const double x2_dot = eta_dotf[0], y2_dot = eta_dotf[1];
        const double psif_dot = follower.r;

        const double l12 = Z[1];
        const double alpha_zero = Z[2];
        const double gama1 = Z[3];

        const double l12_dot = (y2_dot - y1_dot) * sin(alpha_zero) +
                               (x2_dot - x1_dot) * cos(alpha_zero) +
                               d_vessel_ * psif_dot * sin(gama1);

        const double theta_12_dot = ((y2_dot - y1_dot) * cos(alpha_zero) -
                                    (x2_dot - x1_dot) * sin(alpha_zero) +
                                    d_vessel_ * psif_dot * cos(gama1)) / l12 - leader.r;

        const double alpha_zero_dot = leader.r + theta_12_dot;

        return {theta_12_dot, l12_dot, alpha_zero_dot};
    }

    Matrix<double, 2, 3> compute_sliding_surfaces(const std::vector<double>& Z_dot,
                                                  const std::vector<double>& Z_tildas) {
        Matrix<double, 2, 3> surfaces;
        surfaces(0, 0) = Z_dot[0] + Lambda1_ * Z_tildas[0];  // S1
        surfaces(1, 0) = Z_dot[1] + Lambda2_ * Z_tildas[1];  // S2
        surfaces(0, 1) = -Lambda1_ * Z_tildas[0];            // Sr1
        surfaces(1, 1) = -Lambda2_ * Z_tildas[1];            // Sr2
        surfaces(0, 2) = -Lambda1_ * Z_dot[0];               // Sr_dot1
        surfaces(1, 2) = -Lambda2_ * Z_dot[1];               // Sr_dot2
        return surfaces;
    }

    std::vector<double> compute_nonlinear_terms(const State& follower,
                                               const std::vector<double>& Z,
                                               const std::vector<double>& Z_dot) {
        const double l12 = Z[1], gama1 = Z[3];
        const double l12_dot = Z_dot[1], alpha_zero_dot = Z_dot[2];

        const double A2 = (md_ * follower.u * follower.r - D22_ * follower.v)/M22_ +
                         (d_vessel_ * (M33_ * follower.r - md_ * follower.u * follower.v))/M33_;

        const double B2 = (D11_ * follower.u - md_ * follower.v * follower.r)/M11_;

        const double f_theta = A2 * cos(gama1) + B2 * sin(gama1) +
                              d_vessel_ * pow(follower.r, 2) * sin(gama1) -
                              2 * l12_dot * alpha_zero_dot;

        const double f_l = A2 * sin(gama1) - B2 * cos(gama1) -
                          d_vessel_ * pow(follower.r, 2) * cos(gama1) +
                          l12 * pow(alpha_zero_dot, 2);

        return {f_theta, f_l};
    }

    Matrix3d rotation_matrix(double psi) {
        Matrix3d R;
        R << cos(psi), -sin(psi), 0,
             sin(psi),  cos(psi), 0,
             0,         0,        1;
        return R;
    }

    std::vector<double> get_wave_disturbances(const State& follower) {
        return {follower.sigma_u, follower.sigma_v, follower.sigma_r};
    }

    std::vector<double> compute_disturbance_effects(const std::vector<double>& disturbances, const std::vector<double>& Z) {
        const double gama1 = Z[3];
        const double Wu = disturbances[0];
        const double Wv = disturbances[1];
        const double Wr = disturbances[2];

        const double W_theta = (-Wu * sin(gama1) + mr_ * Wv * cos(gama1))/M11_ +
                              (d_vessel_ * Wr * sin(gama1))/M33_;

        const double W_l = (Wu * cos(gama1) + mr_ * Wv * sin(gama1))/M11_ +
                          (d_vessel_ * Wr * cos(gama1))/M33_;

        return {W_theta, W_l};
    }

    

    std::vector<double> compute_control_inputs(const std::vector<double>& fl_fthetha,
        const std::vector<double>& Z,
        const Matrix<double, 2, 3>& S_matrix,
        const std::vector<double>& W) {
        const double l12 = Z[1], gama1 = Z[3];
        const std::vector<double> Sr_dot = {S_matrix(0,2), S_matrix(1,2)};

        // System matrix
        Matrix2d b;
        b << -sin(gama1)/(M11_*l12), d_vessel_*cos(gama1)/(M33_*l12),
              cos(gama1)/M11_,       d_vessel_*sin(gama1)/M33_;

        const double det = b(0,0)*b(1,1) - b(0,1)*b(1,0);
        if (std::abs(det) < 1e-12) {
            RCLCPP_ERROR(get_logger(), "Singular matrix: det=%.3e", det);
            return {0.0, 0.0}; // Return zero control to avoid crash
        }

        // Inverse of the system matrix
        Eigen::Matrix2d inv_b;
        inv_b <<  b(1,1)/det, -b(0,1)/det,
                 -b(1,0)/det,  b(0,0)/det;

        Vector2d f_vec(fl_fthetha[0]/l12, fl_fthetha[1]);
        Vector2d W_vec(W[0], W[1]);
        Vector2d Sr_dot_vec(Sr_dot[0], Sr_dot[1]);
        Vector2d sat_vec(K1_ * tanh(S_matrix(0,0)/phi1_),
                         K2_ * tanh(S_matrix(1,0)/phi2_));

        Vector2d u_vec = inv_b * (-f_vec - W_vec + Sr_dot_vec - sat_vec);

        return {u_vec[0], u_vec[1]};
    }

    void publish_Z(const std::vector<double>& Z) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {Z[0], Z[1], Z[2], Z[3]};
        formation_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published formation parameters:  theta12=%.3f, l12=%.3f, alphaZero=%.3f,gama1=%.3f",
            Z[0], Z[1],Z[2],Z[3]);}

    void publish_control(const std::vector<double>& control_inputs) {
        geometry_msgs::msg::Vector3 msg;
        msg.x = control_inputs[0];
        msg.y = 0.0;
        msg.z = control_inputs[1];
        control_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published control: τ1=%.3f, F2=%.3f", control_inputs[0], control_inputs[1]);
    }

    void publish_errors(const std::vector<double>& Z_tildas) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {Z_tildas[0], Z_tildas[1]};
        error_pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Publishing errors: Theta12_ERROR=%.3f, L12_ERROR=%.3f", Z_tildas[0], Z_tildas[1]);
    }

    void publish_sliding_surfaces(const Matrix<double, 2, 3>& S_matrix) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {S_matrix(0,0), S_matrix(1,0), S_matrix(0,1), S_matrix(1,1),
                    S_matrix(0,2), S_matrix(1,2)};
        slider_pub_->publish(msg);
        RCLCPP_DEBUG(get_logger(), "Published sliding surfaces : s1=%.3f,s2=%.3f,sr1=%.3f,sr2=%.3f,sr_dot_1=%.3f,sr_dot_2=%.3f",S_matrix(0,0),S_matrix(1,0), S_matrix(0,1),S_matrix(1,1),S_matrix(0,2),S_matrix(1,2));
    }

    void publish_wave(const std::vector<double>& wave_disturbances) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {wave_disturbances[0], wave_disturbances[1], wave_disturbances[2]};
        wave_pub_->publish(msg);
        RCLCPP_DEBUG(get_logger(), "Published wave disturbances : w_u=%.3f,w_v=%.3f,w_r=%.3f",wave_disturbances[0],wave_disturbances[1],wave_disturbances[2]);
    }

    void publish_W(const std::vector<double>& W) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {W[0], W[1]};
        W_effect_pub_->publish(msg);
        RCLCPP_DEBUG(get_logger(), "Published disturbance effects : w_l=%.3f,w_thetha=%.3f",W[0], W[1]);
    }

    // ******************** Member Variables ********************
    bool armed = false;
    bool prev_armed = false;
    int arm_counter = 0;
    const int ARM_WAIT_ITER = 7;

    double Ts_;
    double M11_, M22_, M33_, D11_, D22_, D33_, mr_, md_;
    double L12_d_, Theta12_d_, d_vessel_, phi1_, phi2_;
    double K1_, K2_, Lambda1_, Lambda2_;
    
    // Input gain limits
    double IGumax_ff, IGumax_rf, IGumin_rf, IGrmax_ff, IGrmax_rf;
    double Dz_1, Dz_2;
    
    // Deadzone compensation
    double Dz_up_,Dz_down_;

    // Polynomial coefficients for thrust mapping
    float mf0, mf1, mf2, mf3, mf4, mf5;
    float mr0, mr1, mr2, mr3, mr4, mr5;
    float df0, df1, df2, df3, df4, df5;
    float dr0, dr1, dr2, dr3, dr4, dr5;

    State leader_state_;
    State follower_state_;
    std::mutex data_mutex_;
    bool leader_received_ = false;
    bool follower_received_ = false;
    bool controller_started_ = false;
    rclcpp::Time last_leader_time_;
    rclcpp::Time last_follower_time_;

    // Callback group for sensor data
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr arm_disarm_state;
    rclcpp::Subscription<asv_interfaces::msg::StateObserver>::SharedPtr this_vessel_states_obs_;
    rclcpp::Subscription<asv_interfaces::msg::StateNeighbor>::SharedPtr subscriber_state_neighbor_;
    // For this Cnotroller
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr formation_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr control_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr error_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr slider_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wave_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr W_effect_pub_;

    //ASV PWM PUBLISHER
    rclcpp::Publisher<asv_interfaces::msg::PwmValues>::SharedPtr pub_pwm_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_IG_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr test_pub_;
    
    rclcpp::CallbackGroup::SharedPtr cb_group_sensors_;
    rclcpp::CallbackGroup::SharedPtr cb_group_obs_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

    rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LThetaSMCFormation>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
