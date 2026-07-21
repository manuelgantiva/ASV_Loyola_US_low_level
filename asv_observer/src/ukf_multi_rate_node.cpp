#include "rclcpp/rclcpp.hpp"                    // ROS2 C++ API
#include "sensor_msgs/msg/imu.hpp"              // IMU message type
#include "mavros_msgs/msg/state.hpp"            // Armed/disarmed state
#include "std_msgs/msg/float32_multi_array.hpp" // observer/data_sensores
#include "std_msgs/msg/float64_multi_array.hpp" // UKF output state vector
#include "asv_interfaces/msg/state_observer.hpp" // Custom state observer message
#include "rcl_interfaces/msg/set_parameters_result.hpp" // Parameter change callback result


#include <tf2/LinearMath/Quaternion.h>          // Quaternion utilities
#include <tf2/LinearMath/Matrix3x3.h>           // Quaternion -> roll, pitch, yaw


#include <Eigen/Dense>                          // Matrix and vector operations
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>


#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <stdexcept>
#include <algorithm>
#include <mutex>


using std::placeholders::_1;


/*
Before adding #include "asv_interfaces/msg/state_observer.hpp", we were using
Float64MultiArray to publish the state vector. Now we also publish a custom
message specially designed for the state observer with this structure:


std_msgs/Header header
geometry_msgs/Point point
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 disturbances
*/


/*
===============================================================================
MULTI-RATE MODEL-BASED UKF FOR YELLOWFISH
===============================================================================


OVERVIEW
--------
This node implements a model-based Unscented Kalman Filter (UKF) for the
Yellowfish ASV using:


1) High-rate IMU measurements               -> typically 100 Hz
2) Low-rate observer/data_sensores packet   -> typically 10 Hz
3) UKF state output published at IMU rate   -> typically 100 Hz


-------------------------------------------------------------------------------
STATE VECTOR
-------------------------------------------------------------------------------
We estimate the following 12-state vector:


x =
[
  x,
  y,
  psi,
  u,
  v,
  r,
  b_ax,
  b_ay,
  b_gr,
  sigma_u_e,
  sigma_v_e,
  sigma_r_e
]^T


where:


x, y, psi        : global position and heading
u, v, r          : body-frame surge, sway, and yaw rate
b_ax, b_ay       : accelerometer biases
b_gr             : gyroscope bias
sigma_u_e        : unknown/unmodeled surge dynamic term
sigma_v_e        : unknown/unmodeled sway dynamic term
sigma_r_e        : unknown/unmodeled yaw dynamic term


-------------------------------------------------------------------------------
MEASUREMENT MODEL
-------------------------------------------------------------------------------
When the low-rate packet is fresh, the full measurement is:


z_full = [ x, y, psi, ax, ay, r ]^T


When no new low-rate packet is available, only the high-rate IMU is used:


z_imu  = [ ax, ay, r ]^T


-------------------------------------------------------------------------------
PREDICTION MODEL
-------------------------------------------------------------------------------
The model uses the identified input gain terms and the modeled hydrodynamic
disturbances.


Kinematics:
-----------
x_dot   = u*cos(psi) - v*sin(psi)
y_dot   = u*sin(psi) + v*cos(psi)
psi_dot = r


Dynamics:
---------
u_dot = Gu + sigma_u + sigma_u_e
v_dot = Gv + sigma_v + sigma_v_e
r_dot = Gr + sigma_r + sigma_r_e


Then Euler discretization is used:
----------------------------------
x_{k+1} = x_k + dt * x_dot


-------------------------------------------------------------------------------
INPUT GAIN MODEL
-------------------------------------------------------------------------------
The low-rate observer packet contains:


delta_diff, delta_mean, delta_left, delta_right


These are used in the model prediction.


Important:
----------
These delta values are INPUTS of the dynamic model, not measurements.


Therefore:
- they CAN be held constant between low-rate packets (zero-order hold)
- this is exactly what happens automatically in this node


-------------------------------------------------------------------------------
MULTI-RATE UKF LOGIC
-------------------------------------------------------------------------------
At every IMU callback (100 Hz):


1) Use fresh IMU measurement [ax, ay, r]
2) If a NEW low-rate packet arrived, also use [x, y, psi]
3) Correct using the available measurement set
4) Publish the corrected state at 100 Hz
5) Predict one step using the last available delta values


This means:
- output rate = IMU rate
- navigation update rate = low-rate packet rate
- control input is held constant between low-rate packets
===============================================================================
*/


class UnscentedKalmanFilter : public rclcpp::Node
{
public:
  /*
  ============================================================================
  CONSTRUCTOR
  ============================================================================
  In the constructor we do four main things:


  1) Declare parameters
  2) Read parameters
  3) Initialize UKF storage
  4) Create ROS2 subscribers and publishers
  */
  UnscentedKalmanFilter()
  : rclcpp::Node("ukf_multi_rate_node")
  {
    // ------------------------------------------------------------------------
    // 1) DECLARE PARAMETERS
    // ------------------------------------------------------------------------


    // Vehicle / namespace ID
    declare_parameter<std::string>("my_id", "ASV0");


    // IMU sample time in milliseconds
    // Example: Ts = 10 ms => 100 Hz
    declare_parameter<double>("Ts", 10.0);
    declare_parameter<double>("gravity", 9.81);


    // Identified model parameters
    declare_parameter<std::vector<double>>("Xu", std::vector<double>(6, 0.0));
    declare_parameter<std::vector<double>>("Xv", std::vector<double>(12, 0.0));
    declare_parameter<std::vector<double>>("Xr", std::vector<double>(12, 0.0));


    /*
    Diagonal covariance parameters:


    P_init : 12 diagonal values
    Q      : 12 diagonal values
    R      :  6 diagonal values
    */
    declare_parameter<std::vector<double>>("P_init", makeDiagonalIdentity(NX));
    declare_parameter<std::vector<double>>("Q",      makeDiagonalIdentity(NX));
    declare_parameter<std::vector<double>>("R",      makeDiagonalIdentity(NZ_FULL));


    // UKF sigma point tuning parameters
    declare_parameter<double>("alpha", 0.001);
    declare_parameter<double>("beta",  2.0);
    declare_parameter<double>("kappa", -2.0);


    // ------------------------------------------------------------------------
    // 2) READ PARAMETERS
    // ------------------------------------------------------------------------
    my_id_   = get_parameter("my_id").as_string();
    Ts_ms_   = get_parameter("Ts").as_double();
    gravity_ = get_parameter("gravity").as_double();


    // Convert milliseconds -> seconds
    dt_ = Ts_ms_ / 1000.0;


    Xu_ = get_parameter("Xu").as_double_array();
    Xv_ = get_parameter("Xv").as_double_array();
    Xr_ = get_parameter("Xr").as_double_array();


    if (Xu_.size() != 6) {
      throw std::runtime_error("Parameter 'Xu' must have 6 elements.");
    }
    if (Xv_.size() != 12) {
      throw std::runtime_error("Parameter 'Xv' must have 12 elements.");
    }
    if (Xr_.size() != 12) {
      throw std::runtime_error("Parameter 'Xr' must have 12 elements.");
    }


    const auto P_diag = get_parameter("P_init").as_double_array();
    const auto Q_diag = get_parameter("Q").as_double_array();
    const auto R_diag = get_parameter("R").as_double_array();


    P_init_ = vectorToDiagonalMatrix(P_diag, NX);
    Q_      = vectorToDiagonalMatrix(Q_diag, NX);
    R_full_ = vectorToDiagonalMatrix(R_diag, NZ_FULL);

    // adding the rclcpp warn sterm to see wether the filter is recieving the Q an R matrices correctly or not
    RCLCPP_WARN_STREAM(
      get_logger(),
      "Q parameter: " << Eigen::Map<const Eigen::VectorXd>(
        Q_diag.data(), static_cast<Eigen::Index>(Q_diag.size())).transpose());

    RCLCPP_WARN_STREAM(
      get_logger(),
      "R parameter: " << Eigen::Map<const Eigen::VectorXd>(
        R_diag.data(), static_cast<Eigen::Index>(R_diag.size())).transpose());

    RCLCPP_WARN_STREAM(
      get_logger(),
      "Internal Q diagonal: " << Q_.diagonal().transpose());

    RCLCPP_WARN_STREAM(
      get_logger(),
      "Internal R diagonal: " << R_full_.diagonal().transpose());

      RCLCPP_WARN_STREAM(
      get_logger(),
      "Effective Q-step diagonal: "
        << (Q_ * dt_).diagonal().transpose());




    alpha_ = get_parameter("alpha").as_double();
    beta_  = get_parameter("beta").as_double();
    kappa_ = get_parameter("kappa").as_double();


    // ------------------------------------------------------------------------
    // 3) INITIALIZE UKF STORAGE
    // ------------------------------------------------------------------------
    x_hat_       = Eigen::VectorXd::Zero(NX);
    x_posterior_ = Eigen::VectorXd::Zero(NX);
    P_           = P_init_;


    computeUnscentedWeights();


    // ------------------------------------------------------------------------
    // 4) CREATE ROS2 SUBSCRIBERS AND PUBLISHERS
    // ------------------------------------------------------------------------


    // Armed/disarmed state
    subscriber_state_ = create_subscription<mavros_msgs::msg::State>(
      "/" + my_id_ + "/mavros/state",
      rclcpp::QoS(10),
      std::bind(&UnscentedKalmanFilter::callbackStateData, this, _1));


    // IMU topic (fast)
    subscriber_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/" + my_id_ + "/comunication/imu_ext/data",
      rclcpp::SensorDataQoS(),
      std::bind(&UnscentedKalmanFilter::callbackImuData, this, _1));


    // Low-rate observer packet
    subscriber_observer_data_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/" + my_id_ + "/observer/data_sensors",
      rclcpp::SensorDataQoS(),
      std::bind(&UnscentedKalmanFilter::callbackObserverData, this, _1));


    // UKF estimated state output with custom state observer message
    publisher_state_estimate_ = create_publisher<asv_interfaces::msg::StateObserver>(
      "/" + my_id_ + "/observer/state_observer_ukf",
      rclcpp::SensorDataQoS());

    // Full-fusion estimate, published ONLY on cycles where a low-rate packet
    // was incorporated -> effective rate = packet arrival rate (~10 Hz). Used for Controllers that want the best possible state estimate and can handle lower update rates.
    publisher_state_estimate_lowrate_ = create_publisher<asv_interfaces::msg::StateObserver>(
      "/" + my_id_ + "/observer/state_observer_ukf_lowrate",
      rclcpp::SensorDataQoS());
    
    // Full estimated state vector
    publisher_full_state_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + my_id_ + "/observer/state_ukf",
      rclcpp::SensorDataQoS());

    parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&UnscentedKalmanFilter::onParametersChanged,this,std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "UKF started for vehicle: %s", my_id_.c_str());
    RCLCPP_INFO(get_logger(), "IMU dt = %.6f s", dt_);
    RCLCPP_INFO(get_logger(), "observer/data_sensores is assumed low-rate");
    RCLCPP_INFO(get_logger(), "UKF output is published at IMU rate");
  }


private:
  /*
  ============================================================================
  CONSTANTS
  ============================================================================
  */
  static constexpr int NX      = 12; // Number of states
  static constexpr int NZ_FULL = 6;  // Full measurement: [x y psi ax ay r]
  static constexpr int NZ_IMU  = 3;  // IMU-only measurement: [ax ay r]
  

  /*
  ============================================================================
  STATE INDEXING
  ============================================================================
  */
  enum StateIndex
  {
    IDX_X   = 0,  // x position
    IDX_Y   = 1,  // y position
    IDX_PSI = 2,  // heading
    IDX_U   = 3,  // surge velocity
    IDX_V   = 4,  // sway velocity
    IDX_R   = 5,  // yaw rate
    IDX_BAX = 6,  // accel bias x
    IDX_BAY = 7,  // accel bias y
    IDX_BGR = 8,  // gyro bias
    IDX_SEU = 9,  // unmodeled surge term
    IDX_SEV = 10, // unmodeled sway term
    IDX_SER = 11  // unmodeled yaw term
  };


  /*
  ============================================================================
  OPERATING REGION ENUM
  ============================================================================
  */
  enum OperatingRegion
  {
    REGION_FF = 0,
    REGION_FB = 1,
    REGION_BF = 2,
    REGION_BB = 3
  };


  /*
  ============================================================================
  STRUCT: LOW-RATE OBSERVER PACKET
  ============================================================================
  */
  struct ObserverData
  {
    // Navigation measurements
    double x = 0.0;
    double y = 0.0;
    double psi = 0.0;     // wrapped heading used inside the UKF
    double psi_raw = 0.0; // continuous heading used only for final publishing
    double r_low = 0.0;


    // Model inputs
    double delta_diff = 0.0;
    double delta_mean = 0.0;
    double delta_left = 0.0;
    double delta_right = 0.0;


    // Extra packet value
    double beta = 0.0;


    // Computed region based on delta_left and delta_right
    int region = REGION_FF;


    bool valid = false;
    rclcpp::Time stamp;
  };


  /*
  ============================================================================
  STRUCT: IMU DATA AFTER GRAVITY COMPENSATION
  ============================================================================
  */
  struct ImuData
  {
    double ax_body = 0.0;
    double ay_body = 0.0;
    double r_body  = 0.0;


    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;


    bool valid = false;
    rclcpp::Time stamp;
  };


  /*
  ============================================================================
  HELPER: CREATE DIAGONAL IDENTITY PARAMETER VECTOR
  ============================================================================
  */
  static std::vector<double> makeDiagonalIdentity(std::size_t n)
  {
    return std::vector<double>(n, 1.0);
  }


  /*
  ===============================================================================
  PARAMETER VALIDATION HELPERS
  ===============================================================================
  */
  static bool vectorIsFinite(const std::vector<double> & values)
  {
    return std::all_of(
      values.begin(),
      values.end(),
      [](double value)
      {
        return std::isfinite(value);
      });
  }


  static bool vectorIsNonNegative(const std::vector<double> & values)
  {
    return std::all_of(
      values.begin(),
      values.end(),
      [](double value)
      {
        return std::isfinite(value) && value >= 0.0;
      });
  }


  static bool vectorIsStrictlyPositive(const std::vector<double> & values)
  {
    return std::all_of(
      values.begin(),
      values.end(),
      [](double value)
      {
        return std::isfinite(value) && value > 0.0;
      });
  }
    /*
  ===============================================================================
  RUNTIME PARAMETER CALLBACK
  ===============================================================================

  Parameters handled at runtime:

    Ts, gravity,
    Xu, Xv, Xr,
    P_init, Q, R,
    alpha, beta, kappa

  The callback first creates candidate values and validates the entire request.
  Only after everything is valid are the internal UKF variables changed.
  */
  rcl_interfaces::msg::SetParametersResult onParametersChanged(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "Parameter validation failed";

    // --------------------------------------------------------------------------
    // Candidate values
    // --------------------------------------------------------------------------
    double new_Ts_ms   = Ts_ms_;
    double new_gravity = gravity_;

    std::vector<double> new_Xu = Xu_;
    std::vector<double> new_Xv = Xv_;
    std::vector<double> new_Xr = Xr_;

    Eigen::MatrixXd new_P_init = P_init_;
    Eigen::MatrixXd new_Q      = Q_;
    Eigen::MatrixXd new_R      = R_full_;

    double new_alpha = alpha_;
    double new_beta  = beta_;
    double new_kappa = kappa_;

    // Keep track of which values were requested.
    bool change_Ts       = false;
    bool change_gravity  = false;
    bool change_Xu       = false;
    bool change_Xv       = false;
    bool change_Xr       = false;
    bool change_P_init   = false;
    bool change_Q        = false;
    bool change_R        = false;
    bool change_alpha    = false;
    bool change_beta     = false;
    bool change_kappa    = false;

    // --------------------------------------------------------------------------
    // Read and validate parameter types, sizes, and individual values
    // --------------------------------------------------------------------------
    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();

      // my_id cannot safely be changed because all topics were already created.
      if (name == "my_id") {
        result.reason =
          "Parameter 'my_id' cannot be changed at runtime because publishers "
          "and subscribers have already been created.";
        return result;
      }

      if (name == "Ts") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "'Ts' must be a double, for example 10.0";
          return result;
        }

        new_Ts_ms = parameter.as_double();

        if (!std::isfinite(new_Ts_ms) || new_Ts_ms <= 0.0) {
          result.reason = "'Ts' must be finite and greater than zero";
          return result;
        }

        change_Ts = true;
      }

      else if (name == "gravity") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "'gravity' must be a double";
          return result;
        }

        new_gravity = parameter.as_double();

        if (!std::isfinite(new_gravity) || new_gravity <= 0.0) {
          result.reason = "'gravity' must be finite and greater than zero";
          return result;
        }

        change_gravity = true;
      }

      else if (name == "Xu") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'Xu' must be a double array";
          return result;
        }

        new_Xu = parameter.as_double_array();

        if (new_Xu.size() != 6) {
          result.reason = "'Xu' must contain exactly 6 values";
          return result;
        }

        if (!vectorIsFinite(new_Xu)) {
          result.reason = "Every value in 'Xu' must be finite";
          return result;
        }

        change_Xu = true;
      }

      else if (name == "Xv") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'Xv' must be a double array";
          return result;
        }

        new_Xv = parameter.as_double_array();

        if (new_Xv.size() != 12) {
          result.reason = "'Xv' must contain exactly 12 values";
          return result;
        }

        if (!vectorIsFinite(new_Xv)) {
          result.reason = "Every value in 'Xv' must be finite";
          return result;
        }

        change_Xv = true;
      }

      else if (name == "Xr") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'Xr' must be a double array";
          return result;
        }

        new_Xr = parameter.as_double_array();

        if (new_Xr.size() != 12) {
          result.reason = "'Xr' must contain exactly 12 values";
          return result;
        }

        if (!vectorIsFinite(new_Xr)) {
          result.reason = "Every value in 'Xr' must be finite";
          return result;
        }

        change_Xr = true;
      }

      else if (name == "P_init") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'P_init' must be a double array";
          return result;
        }

        const std::vector<double> values =
          parameter.as_double_array();

        if (values.size() != NX) {
          result.reason = "'P_init' must contain exactly 12 values";
          return result;
        }

        if (!vectorIsNonNegative(values)) {
          result.reason =
            "Every value in 'P_init' must be finite and nonnegative";
          return result;
        }

        new_P_init = vectorToDiagonalMatrix(values, NX);
        change_P_init = true;
      }

      else if (name == "Q") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'Q' must be a double array";
          return result;
        }

        const std::vector<double> values =
          parameter.as_double_array();

        if (values.size() != NX) {
          result.reason = "'Q' must contain exactly 12 values";
          return result;
        }

        if (!vectorIsNonNegative(values)) {
          result.reason =
            "Every value in 'Q' must be finite and nonnegative";
          return result;
        }

        new_Q = vectorToDiagonalMatrix(values, NX);
        change_Q = true;
      }

      else if (name == "R") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          result.reason = "'R' must be a double array";
          return result;
        }

        const std::vector<double> values =
          parameter.as_double_array();

        if (values.size() != NZ_FULL) {
          result.reason =
            "'R' must contain exactly 6 values: [x,y,psi,ax,ay,r]";
          return result;
        }

        if (!vectorIsStrictlyPositive(values)) {
          result.reason =
            "Every value in 'R' must be finite and greater than zero";
          return result;
        }

        new_R = vectorToDiagonalMatrix(values, NZ_FULL);
        change_R = true;
      }

      else if (name == "alpha") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "'alpha' must be a double";
          return result;
        }

        new_alpha = parameter.as_double();

        if (!std::isfinite(new_alpha) || new_alpha <= 0.0) {
          result.reason = "'alpha' must be finite and greater than zero";
          return result;
        }

        change_alpha = true;
      }

      else if (name == "beta") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "'beta' must be a double";
          return result;
        }

        new_beta = parameter.as_double();

        if (!std::isfinite(new_beta) || new_beta < 0.0) {
          result.reason = "'beta' must be finite and nonnegative";
          return result;
        }

        change_beta = true;
      }

      else if (name == "kappa") {
        if (parameter.get_type() !=
            rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          result.reason = "'kappa' must be a double";
          return result;
        }

        new_kappa = parameter.as_double();

        if (!std::isfinite(new_kappa)) {
          result.reason = "'kappa' must be finite";
          return result;
        }

        change_kappa = true;
      }
    }

    // --------------------------------------------------------------------------
    // Validate the combined UKF sigma-point parameters
    // --------------------------------------------------------------------------
    //
    // NX + lambda = alpha^2 * (NX + kappa)
    //
    // Therefore NX + kappa must be positive.
    if ((static_cast<double>(NX) + new_kappa) <= 0.0) {
      result.reason =
        "'kappa' is invalid: NX + kappa must be greater than zero. "
        "For NX=12, kappa must be greater than -12.";
      return result;
    }

    const double new_lambda = new_alpha * new_alpha * (static_cast<double>(NX) + new_kappa) - static_cast<double>(NX);

    const double new_scaling = static_cast<double>(NX) + new_lambda;

    if (!std::isfinite(new_lambda) || !std::isfinite(new_scaling) || new_scaling <= 0.0)
    {
      result.reason =
        "Invalid alpha/kappa combination: NX + lambda must be positive";
      return result;
    }

    // --------------------------------------------------------------------------
    // Apply all validated changes
    // --------------------------------------------------------------------------
    if (change_Ts) {
      Ts_ms_ = new_Ts_ms;
      dt_ = Ts_ms_ / 1000.0;

      RCLCPP_WARN(
        get_logger(),
        "Runtime Ts updated: Ts=%.6f ms, dt=%.9f s",
        Ts_ms_, dt_);
    }

    if (change_gravity) {
      gravity_ = new_gravity;

      RCLCPP_WARN(
        get_logger(),
        "Runtime gravity updated: %.9f m/s^2",
        gravity_);
    }

    if (change_Xu) {
      Xu_ = new_Xu;

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime Xu updated: "
          << Eigen::Map<const Eigen::VectorXd>(
              Xu_.data(),
              static_cast<Eigen::Index>(Xu_.size())).transpose());
    }

    if (change_Xv) {
      Xv_ = new_Xv;

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime Xv updated: "
          << Eigen::Map<const Eigen::VectorXd>(
              Xv_.data(),
              static_cast<Eigen::Index>(Xv_.size())).transpose());
    }

    if (change_Xr) {
      Xr_ = new_Xr;

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime Xr updated: "
          << Eigen::Map<const Eigen::VectorXd>(
              Xr_.data(),
              static_cast<Eigen::Index>(Xr_.size())).transpose());
    }

    if (change_P_init) {
      P_init_ = new_P_init;

      /*
      P_init controls filter initialization.

      Do not reset the covariance of a filter that is currently operating.
      When the filter is not initialized, update P_ immediately.
      Otherwise, the new P_init is used after disarm/rearm or another reset.
      */
      if (!initialized_) {
        P_ = P_init_;
      }

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime P_init updated: "
          << P_init_.diagonal().transpose()
          << (initialized_
                ? " — will be applied at the next UKF initialization"
                : " — applied immediately because UKF is not initialized"));
    }

    if (change_Q) {
      Q_ = new_Q;

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime Q updated: "
          << Q_.diagonal().transpose()
          << " | effective Q*dt: "
          << (Q_ * dt_).diagonal().transpose());
    }

    if (change_R) {
      R_full_ = new_R;

      RCLCPP_WARN_STREAM(
        get_logger(),
        "Runtime R updated: "
          << R_full_.diagonal().transpose()
          << " | order=[x y psi ax ay r]");
    }

    if (change_alpha || change_beta || change_kappa) {
      alpha_ = new_alpha;
      beta_  = new_beta;
      kappa_ = new_kappa;

      computeUnscentedWeights();

      RCLCPP_WARN_STREAM(
        get_logger(),
        "UKF scaling updated:"
          << " alpha=" << alpha_
          << " beta=" << beta_
          << " kappa=" << kappa_
          << " lambda=" << lambda_
          << " NX+lambda=" << (static_cast<double>(NX) + lambda_)
          << " gamma=" << gamma_
          << " Wm0=" << Wm_(0)
          << " Wc0=" << Wc_(0));
    }

    result.successful = true;
    result.reason = "Runtime UKF parameters updated successfully";
    return result;
  }

  /*
  ============================================================================
  HELPER: CONVERT VECTOR TO DIAGONAL EIGEN MATRIX
  ============================================================================
  */
  static Eigen::MatrixXd vectorToDiagonalMatrix(const std::vector<double> & data, int size)
  {
    if (static_cast<int>(data.size()) != size) {
      throw std::runtime_error("Diagonal matrix parameter has wrong size.");
    }


    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(size, size);
    for (int i = 0; i < size; ++i) {
      M(i, i) = data[static_cast<std::size_t>(i)];
    }


    return M;
  }


  /*
  ============================================================================
  HELPER: WRAP ANGLE TO [-pi, pi]
  ============================================================================
  */
  static double wrapAngle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  /*
  ============================================================================
  HELPER: Double WRAP ANGLE to Publish heading continuously
  ============================================================================
  */
  static double unwrapAngleNear(double wrapped_angle, double reference_angle)
  {
    static constexpr double TWO_PI = 2.0 * M_PI;
    return wrapped_angle + TWO_PI * std::round((reference_angle - wrapped_angle) / TWO_PI);
  }

  /*
  ============================================================================
  HELPER: DETERMINE REGION FROM LEFT/RIGHT THRUSTER SIGNS
  ============================================================================
  */
  int determineRegion(double delta_left, double delta_right) const
  {
    if (delta_left >= 0.0 && delta_right >= 0.0) {
      return REGION_FF;
    } else if (delta_left >= 0.0 && delta_right < 0.0) {
      return REGION_FB;
    } else if (delta_left < 0.0 && delta_right >= 0.0) {
      return REGION_BF;
    } else {
      return REGION_BB;
    }
  }


  /*
  ============================================================================
  UKF WEIGHT COMPUTATION
  ============================================================================
  */
  void computeUnscentedWeights()
  {
    lambda_ = alpha_ * alpha_ * (NX + kappa_) - NX;


    if ((NX + lambda_) <= 0.0) {
      throw std::runtime_error("Invalid UKF parameters: NX + lambda_ must be positive.");
    }


    gamma_ = std::sqrt(NX + lambda_);


    Wm_ = Eigen::VectorXd::Constant(2 * NX + 1, 1.0 / (2.0 * (NX + lambda_)));
    Wc_ = Eigen::VectorXd::Constant(2 * NX + 1, 1.0 / (2.0 * (NX + lambda_)));


    Wm_(0) = lambda_ / (NX + lambda_);
    Wc_(0) = lambda_ / (NX + lambda_) + (1.0 - alpha_ * alpha_ + beta_);
  }


  /*
  ============================================================================
  MATRIX SQUARE ROOT
  ============================================================================
  */
  Eigen::MatrixXd computeMatrixSquareRoot(const Eigen::MatrixXd & P_in) const
  {
    Eigen::MatrixXd P_sym = 0.5 * (P_in + P_in.transpose());


    Eigen::LLT<Eigen::MatrixXd> llt(P_sym);
    if (llt.info() == Eigen::Success) {
      return llt.matrixL();
    }


    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(P_sym);
    if (es.info() != Eigen::Success) {
      throw std::runtime_error("Failed to compute covariance square root.");
    }


    Eigen::VectorXd evals = es.eigenvalues().cwiseMax(1e-12);
    return es.eigenvectors() * evals.cwiseSqrt().asDiagonal();
  }


  /*
  ============================================================================
  GENERATE SIGMA POINTS
  ============================================================================
  */
  Eigen::MatrixXd generateSigmaPoints(const Eigen::VectorXd & x, const Eigen::MatrixXd & P) const
  {
    Eigen::MatrixXd sigma_points(NX, 2 * NX + 1);
    sigma_points.col(0) = x;


    const Eigen::MatrixXd sqrtP = computeMatrixSquareRoot(P);


    for (int i = 0; i < NX; ++i) {
      sigma_points.col(i + 1)      = x + gamma_ * sqrtP.col(i);
      sigma_points.col(i + 1 + NX) = x - gamma_ * sqrtP.col(i);


      sigma_points(IDX_PSI, i + 1)      = wrapAngle(sigma_points(IDX_PSI, i + 1));
      sigma_points(IDX_PSI, i + 1 + NX) = wrapAngle(sigma_points(IDX_PSI, i + 1 + NX));
    }


    return sigma_points;
  }


  /*
  ============================================================================
  CALLBACK: ARMED / DISARMED STATE
  ============================================================================
  */
  void callbackStateData(const mavros_msgs::msg::State::SharedPtr msg)
  {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Received armed state: %s", msg->armed ? "ARMED" : "DISARMED");


    const bool was_armed = armed_;
    armed_ = msg->armed;


    if (was_armed && !armed_) {
      std::lock_guard<std::mutex> lock(data_mutex_);


      initialized_ = false;
      x_hat_ = Eigen::VectorXd::Zero(NX);
      x_posterior_ = Eigen::VectorXd::Zero(NX);
      P_ = P_init_;


      have_fresh_low_rate_ = false;
      low_rate_update_this_cycle_ = false;


      observer_latest_.x = 0.0;
      observer_latest_.y = 0.0;
      observer_latest_.psi = 0.0;
      observer_latest_.r_low = 0.0;
      observer_latest_.delta_diff = 0.0;
      observer_latest_.delta_mean = 0.0;
      observer_latest_.delta_left = 0.0;
      observer_latest_.delta_right = 0.0;
      observer_latest_.beta = 0.0;
      observer_latest_.region = REGION_FF;
      observer_latest_.valid = false;
      observer_latest_.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);


      observer_cycle_.x = 0.0;
      observer_cycle_.y = 0.0;
      observer_cycle_.psi = 0.0;
      observer_cycle_.r_low = 0.0;
      observer_cycle_.delta_diff = 0.0;
      observer_cycle_.delta_mean = 0.0;
      observer_cycle_.delta_left = 0.0;
      observer_cycle_.delta_right = 0.0;
      observer_cycle_.beta = 0.0;
      observer_cycle_.region = REGION_FF;
      observer_cycle_.valid = false;
      observer_cycle_.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
  }


  /*
  ============================================================================
  CALLBACK: observer/data_sensores
  ============================================================================
  Expected order:
    [0] x
    [1] y
    [2] psi
    [3] r_low
    [4] delta_diff
    [5] delta_mean
    [6] beta
    [7] delta_left
    [8] delta_right
  */
  void callbackObserverData(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 9) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "observer/data_sensores must contain at least 9 values");
      return;
    }


    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Received low-rate observer packet with x=%.2f, y=%.2f, psi=%.2f, r_low=%.2f, "
      "delta_diff=%.2f, delta_mean=%.2f, beta=%.2f, delta_left=%.2f, delta_right=%.2f",
      msg->data[0], msg->data[1], msg->data[2], msg->data[3],
      msg->data[4], msg->data[5], msg->data[6], msg->data[7], msg->data[8]);


    std::lock_guard<std::mutex> lock(data_mutex_);


    observer_latest_.x           = static_cast<double>(msg->data[0]);
    observer_latest_.y           = static_cast<double>(msg->data[1]);
    // observer_latest_.psi         = wrapAngle(static_cast<double>(msg->data[2]));
    observer_latest_.psi_raw     = static_cast<double>(msg->data[2]);
    observer_latest_.psi         = wrapAngle(observer_latest_.psi_raw);
    observer_latest_.r_low       = static_cast<double>(msg->data[3]);
    observer_latest_.delta_diff  = static_cast<double>(msg->data[4]);
    observer_latest_.delta_mean  = static_cast<double>(msg->data[5]);
    observer_latest_.beta        = static_cast<double>(msg->data[6]);
    observer_latest_.delta_left  = static_cast<double>(msg->data[7]);
    observer_latest_.delta_right = static_cast<double>(msg->data[8]);


    observer_latest_.region = determineRegion(
      observer_latest_.delta_left,
      observer_latest_.delta_right);


    observer_latest_.valid = true;
    observer_latest_.stamp = now();


    // This flag is consumed once in the next IMU cycle.
    have_fresh_low_rate_ = true;
  }


  /*
  ============================================================================
  IMU COMPENSATION
  ============================================================================
  */
  ImuData compensateImu(const sensor_msgs::msg::Imu::SharedPtr msg) const
  {
    ImuData out;
    out.stamp = rclcpp::Time(msg->header.stamp);


    const double qx = msg->orientation.x;
    const double qy = msg->orientation.y;
    const double qz = msg->orientation.z;
    const double qw = msg->orientation.w;


    const double qnorm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (qnorm < 1e-12) {
      throw std::runtime_error("Received IMU quaternion with near-zero norm.");
    }


    tf2::Quaternion q_tf(qx / qnorm, qy / qnorm, qz / qnorm, qw / qnorm);


    double roll_raw, pitch_raw, yaw_raw;
    tf2::Matrix3x3(q_tf).getRPY(roll_raw, pitch_raw, yaw_raw);


    // Sign convention from current implementation
    const double ax_like = msg->linear_acceleration.x;
    const double ay_like = -msg->linear_acceleration.y;
    const double az_like = -msg->linear_acceleration.z;
    const double r_like  = -msg->angular_velocity.z;


    const double roll  = roll_raw;
    const double pitch = -pitch_raw;
    const double yaw   = -yaw_raw;


    /*
    Gravity projection in body frame:


      g_x =  gravity * sin(pitch)
      g_y = -gravity * sin(roll) * cos(pitch)
    */
    const double g_x =  gravity_ * std::sin(pitch);
    const double g_y = -gravity_ * std::sin(roll) * std::cos(pitch);
    const double g_z = -gravity_ * std::cos(roll) * std::cos(pitch);


    out.ax_body = ax_like - g_x;
    out.ay_body = ay_like - g_y;


    (void)az_like;
    (void)g_z;


    out.r_body = r_like;
    out.roll   = roll;
    out.pitch  = pitch;
    out.yaw    = yaw;
    out.valid  = true;


    return out;
  }


  /*
  ============================================================================
  CALLBACK: FAST IMU
  ============================================================================
  */
  void callbackImuData(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    if (!armed_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "System not armed yet. Waiting to start UKF.");
      return;
    }


    ImuData imu_local;


    try {
      imu_local = compensateImu(msg);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "IMU compensation failed: %s", e.what());
      return;
    }


    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "IMU compensation done: ax=%.2f, ay=%.2f, r=%.2f, roll=%.2f, pitch=%.2f, yaw=%.2f",
      imu_local.ax_body, imu_local.ay_body, imu_local.r_body,
      imu_local.roll, imu_local.pitch, imu_local.yaw);


    initializeIfNeeded();
    if (!initialized_) {
      return;
    }


    runFilterCycle(imu_local);
    publishStateEstimate();
  }


  /*
  ============================================================================
  INITIALIZATION
  ============================================================================
  On first run, initialize x, y, psi from the last low-rate packet if available.
  Also initialize yaw rate from r_low.
  */
  void initializeIfNeeded()
  {
    if (initialized_) {
      return;
    }


    std::lock_guard<std::mutex> lock(data_mutex_);


    if (!observer_latest_.valid || !have_fresh_low_rate_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "UKF not initialized yet. observer_valid=%s, have_fresh_low_rate=%s",
        observer_latest_.valid ? "true" : "false",
        have_fresh_low_rate_ ? "true" : "false");
      return;
    }


    x_hat_ = Eigen::VectorXd::Zero(NX);
    x_hat_(IDX_X)   = observer_latest_.x;
    x_hat_(IDX_Y)   = observer_latest_.y;
    x_hat_(IDX_PSI) = observer_latest_.psi;
    x_hat_(IDX_R)   = observer_latest_.r_low;


    P_ = P_init_;
    x_posterior_ = x_hat_;
    initialized_ = true;


    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "UKF initialized with x=%.2f, y=%.2f, psi=%.2f, r=%.2f",
      x_hat_(IDX_X), x_hat_(IDX_Y), x_hat_(IDX_PSI), x_hat_(IDX_R));
  }


  /*
  ============================================================================
  ONE COMPLETE FILTER CYCLE
  ============================================================================
  */
  void runFilterCycle(const ImuData & imu_local)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      observer_cycle_ = observer_latest_;
      low_rate_update_this_cycle_ = have_fresh_low_rate_;
      have_fresh_low_rate_ = false;
    }


    // ------------------------------------------------------------------------
    // 1) UPDATE: use only one measurement set per IMU cycle
    // ------------------------------------------------------------------------
    if (low_rate_update_this_cycle_ && observer_cycle_.valid) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Running UKF update with FULL measurement [x y psi ax ay r]: "
        "x=%.2f, y=%.2f, psi=%.2f, ax=%.2f, ay=%.2f, r=%.2f",
        observer_cycle_.x, observer_cycle_.y, observer_cycle_.psi,
        imu_local.ax_body, imu_local.ay_body, imu_local.r_body);


      Eigen::VectorXd z_full(NZ_FULL);
      z_full << observer_cycle_.x,
                observer_cycle_.y,
                observer_cycle_.psi,
                imu_local.ax_body,
                imu_local.ay_body,
                imu_local.r_body;


      updateStep(z_full, true);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Running UKF update with only IMU measurements [ax ay r]: "
        "ax=%.2f, ay=%.2f, r=%.2f",
        imu_local.ax_body, imu_local.ay_body, imu_local.r_body);


      Eigen::VectorXd z_imu(NZ_IMU);
      z_imu << imu_local.ax_body,
               imu_local.ay_body,
               imu_local.r_body;


      updateStep(z_imu, false);
    }


    // ------------------------------------------------------------------------
    // 2) STORE CORRECTED STATE FOR PUBLISHING
    // ------------------------------------------------------------------------
    x_posterior_ = x_hat_;

 

    // build the the continous heading only
    updateContinuousPsi();

       // Publish the full-fusion estimate ONLY on cycles where the low-rate packet
    // was actually fused. x_posterior_ here is the state just corrected with
    // [x y psi ax ay r], before the next prediction runs.
    if (low_rate_update_this_cycle_ && observer_cycle_.valid) {
      publishLowRateStateEstimate(imu_local);
    }




    // ------------------------------------------------------------------------
    // 3) PREDICT ONE STEP USING HELD delta_* INPUTS
    // ------------------------------------------------------------------------
    predictionStep();
  }


  /*
  ============================================================================
  COMPUTE INPUT GAIN AND MODELED DISTURBANCES
  ============================================================================
  */
  void computeInputGainAndDisturbances(
    const Eigen::VectorXd & x,
    Eigen::Vector3d & Gm,
    Eigen::Vector3d & sigma_m) const
  {
    const double u = x(IDX_U);
    const double v = x(IDX_V);
    const double r = x(IDX_R);


    /*
    observer_cycle_ is copied once at the start of the IMU cycle.
    If no new low-rate packet arrives, observer_cycle_ still contains the
    previous delta values, so the model naturally applies zero-order hold.
    */
    const double delta_mean =
      observer_cycle_.valid ? observer_cycle_.delta_mean : 0.0;


    const double delta_diff =
      observer_cycle_.valid ? observer_cycle_.delta_diff : 0.0;


    const int region =
      observer_cycle_.valid ? observer_cycle_.region : REGION_FF;


    // q = delta_mean^2 + delta_diff^2 / 4
    const double quad = delta_mean * delta_mean + 0.25 * delta_diff * delta_diff;


    // Input-gain coefficients
    const double au = Xu_.at(4);
    const double bu = Xu_.at(5);


    const double av = Xv_.at(8);
    const double bv = Xv_.at(9);
    const double cv = Xv_.at(10);
    const double dv = Xv_.at(11);


    const double ar = Xr_.at(8);
    const double br = Xr_.at(9);
    const double cr = Xr_.at(10);
    const double dr = Xr_.at(11);


    // Surge input gain
    const double Gu = au * quad + bu * delta_mean;


    // Sway and yaw input gains
    double Gv = 0.0;
    double Gr = 0.0;


    switch (region) {
      case REGION_FF:
        Gv = bv * delta_mean * delta_diff + dv * (0.5 * delta_diff);
        Gr = br * delta_mean * delta_diff + dr * (0.5 * delta_diff);
        break;


      case REGION_FB:
        Gv =  av * quad + bv * delta_mean * delta_diff + cv * delta_mean + dv * (0.5 * delta_diff);
        Gr =  ar * quad + br * delta_mean * delta_diff + cr * delta_mean + dr * (0.5 * delta_diff);
        break;


      case REGION_BF:
        Gv = -av * quad + bv * delta_mean * delta_diff - cv * delta_mean + dv * (0.5 * delta_diff);
        Gr = -ar * quad + br * delta_mean * delta_diff - cr * delta_mean + dr * (0.5 * delta_diff);
        break;


      case REGION_BB:
      default:
        Gv = bv * delta_mean * delta_diff + dv * (0.5 * delta_diff);
        Gr = br * delta_mean * delta_diff + dr * (0.5 * delta_diff);
        break;
    }


    Gm << Gu, Gv, Gr;


    // Modeled hydrodynamic disturbance terms
    const double sigma_u =
      Xu_.at(0) * u * std::abs(u) +
      Xu_.at(1) * v * r +
      Xu_.at(2) * r * r +
      Xu_.at(3) * u;


    const double sigma_v =
      Xv_.at(0) * v * std::abs(v) +
      Xv_.at(1) * u * std::abs(r) +
      Xv_.at(2) * r * std::abs(v) +
      Xv_.at(3) * r * std::abs(r) +
      Xv_.at(4) * u * v +
      Xv_.at(5) * u * r +
      Xv_.at(6) * v +
      Xv_.at(7) * r;


    const double sigma_r =
      Xr_.at(0) * v * std::abs(v) +
      Xr_.at(1) * v * std::abs(r) +
      Xr_.at(2) * r * std::abs(v) +
      Xr_.at(3) * r * std::abs(r) +
      Xr_.at(4) * u * v +
      Xr_.at(5) * u * r +
      Xr_.at(6) * v +
      Xr_.at(7) * r;


    sigma_m << sigma_u, sigma_v, sigma_r;
  }


  /*
  ============================================================================
  STATE TRANSITION MODEL
  ============================================================================
  */
  Eigen::VectorXd stateTransition(const Eigen::VectorXd & x) const
  {
    Eigen::Vector3d Gm;
    Eigen::Vector3d sigma_m;
    computeInputGainAndDisturbances(x, Gm, sigma_m);


    const double psi = x(IDX_PSI);
    const double u   = x(IDX_U);
    const double v   = x(IDX_V);
    const double r   = x(IDX_R);


    Eigen::VectorXd x_next = x;


    // Kinematics
    const double x_dot   = u * std::cos(psi) - v * std::sin(psi);
    const double y_dot   = u * std::sin(psi) + v * std::cos(psi);
    const double psi_dot = r;


    // Dynamics
    const double u_dot = Gm(0) + sigma_m(0) + x(IDX_SEU);
    const double v_dot = Gm(1) + sigma_m(1) + x(IDX_SEV);
    const double r_dot = Gm(2) + sigma_m(2) + x(IDX_SER);


    // Euler step
    x_next(IDX_X)   = x(IDX_X)   + dt_ * x_dot;
    x_next(IDX_Y)   = x(IDX_Y)   + dt_ * y_dot;
    x_next(IDX_PSI) = wrapAngle(x(IDX_PSI) + dt_ * psi_dot);
    x_next(IDX_U)   = x(IDX_U)   + dt_ * u_dot;
    x_next(IDX_V)   = x(IDX_V)   + dt_ * v_dot;
    x_next(IDX_R)   = x(IDX_R)   + dt_ * r_dot;


    // Random-walk states keep their deterministic mean
    x_next(IDX_BAX) = x(IDX_BAX);
    x_next(IDX_BAY) = x(IDX_BAY);
    x_next(IDX_BGR) = x(IDX_BGR);
    x_next(IDX_SEU) = x(IDX_SEU);
    x_next(IDX_SEV) = x(IDX_SEV);
    x_next(IDX_SER) = x(IDX_SER);


    return x_next;
  }


  /*
  ============================================================================
  MEASUREMENT MODEL
  ============================================================================
  Full measurement:
      z_full = [x, y, psi, ax, ay, r]^T


  IMU-only measurement:
      z_imu = [ax, ay, r]^T


  The acceleration-level model is:
      ax_model = u_dot - r*v + b_ax
      ay_model = v_dot + r*u + b_ay
      r_model  = r + b_gr
  */
  Eigen::VectorXd measurementModel(const Eigen::VectorXd & x, bool use_full) const
  {
    Eigen::Vector3d Gm;
    Eigen::Vector3d sigma_m;
    computeInputGainAndDisturbances(x, Gm, sigma_m);


    // const double ax_model = Gm(0) + sigma_m(0) + x(IDX_SEU) - x(IDX_R) * x(IDX_V) + x(IDX_BAX);
    // const double ay_model = Gm(1) + sigma_m(1) + x(IDX_SEV) + x(IDX_R) * x(IDX_U) + x(IDX_BAY);
    // const double r_model  = x(IDX_R) + x(IDX_BGR);
    const double ax_model = Gm(0) + sigma_m(0) + x(IDX_SEU) + x(IDX_BAX);
    const double ay_model = Gm(1) + sigma_m(1) + x(IDX_SEV) + x(IDX_BAY);
    const double r_model  = x(IDX_R) + x(IDX_BGR);

    if (use_full) {
      Eigen::VectorXd z(NZ_FULL);
      z << x(IDX_X),
           x(IDX_Y),
           x(IDX_PSI),
           ax_model,
           ay_model,
           r_model;
      return z;
    }


    Eigen::VectorXd z(NZ_IMU);
    z << ax_model,
         ay_model,
         r_model;
    return z;
  }


  /*
  ============================================================================
  PREDICTION STEP
  ============================================================================
  */
  void predictionStep()
  {
    const Eigen::MatrixXd Xsig = generateSigmaPoints(x_hat_, P_);


    Eigen::MatrixXd Xpred(NX, 2 * NX + 1);
    for (int i = 0; i < 2 * NX + 1; ++i) {
      Xpred.col(i) = stateTransition(Xsig.col(i));
    }


    // Predicted mean
    Eigen::VectorXd x_pred = Xpred * Wm_;


    // Circular mean for psi
    double s = 0.0;
    double c = 0.0;
    for (int i = 0; i < 2 * NX + 1; ++i) {
      s += Wm_(i) * std::sin(Xpred(IDX_PSI, i));
      c += Wm_(i) * std::cos(Xpred(IDX_PSI, i));
    }
    x_pred(IDX_PSI) = std::atan2(s, c);


    // Predicted covariance
    Eigen::MatrixXd P_pred = Eigen::MatrixXd::Zero(NX, NX);
    for (int i = 0; i < 2 * NX + 1; ++i) {
      Eigen::VectorXd dx = Xpred.col(i) - x_pred;
      dx(IDX_PSI) = wrapAngle(dx(IDX_PSI));
      P_pred += Wc_(i) * (dx * dx.transpose());
    }


    // Add process noise
    P_pred += Q_ * dt_;


    // Numerical symmetry protection
    P_pred = 0.5 * (P_pred + P_pred.transpose());


    x_hat_ = x_pred;
    P_     = P_pred;
  }


  /*
  ============================================================================
  UPDATE STEP
  ============================================================================
  */
  void updateStep(const Eigen::VectorXd & z, bool use_full)
  {
    const int nz = static_cast<int>(z.size());

    Eigen::MatrixXd R_use;

    if (use_full) {
      // [x, y, psi, ax, ay, r]
      R_use = R_full_;
    } else {
      // IMU-only: [ax, ay, r]
      R_use = R_full_.block<3, 3>(3, 3);
    }

    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "UKF update mode=" << (use_full ? "FULL" : "IMU_ONLY")
        << " | R_used=" << R_use.diagonal().transpose());


    const Eigen::MatrixXd Xsig = generateSigmaPoints(x_hat_, P_);


    // Map sigma points to measurement space
    Eigen::MatrixXd Zsig(nz, 2 * NX + 1);
    for (int i = 0; i < 2 * NX + 1; ++i) {
      Zsig.col(i) = measurementModel(Xsig.col(i), use_full);
    }


    // Predicted measurement mean
    Eigen::VectorXd z_pred = Zsig * Wm_;


    if (use_full) {
      double s = 0.0;
      double c = 0.0;
      for (int i = 0; i < 2 * NX + 1; ++i) {
        s += Wm_(i) * std::sin(Zsig(2, i));
        c += Wm_(i) * std::cos(Zsig(2, i));
      }
      z_pred(2) = std::atan2(s, c);
    }


    // Innovation covariance and cross covariance
    Eigen::MatrixXd Pzz = Eigen::MatrixXd::Zero(nz, nz);
    Eigen::MatrixXd Pxz = Eigen::MatrixXd::Zero(NX, nz);


    for (int i = 0; i < 2 * NX + 1; ++i) {
      Eigen::VectorXd dx = Xsig.col(i) - x_hat_;
      dx(IDX_PSI) = wrapAngle(dx(IDX_PSI));


      Eigen::VectorXd dz = Zsig.col(i) - z_pred;
      if (use_full) {
        dz(2) = wrapAngle(dz(2));
      }


      Pzz += Wc_(i) * (dz * dz.transpose());
      Pxz += Wc_(i) * (dx * dz.transpose());
    }


    Pzz += R_use;
    Pzz = 0.5 * (Pzz + Pzz.transpose());


    // Kalman gain
    const Eigen::MatrixXd K =
      Pxz * Pzz.ldlt().solve(Eigen::MatrixXd::Identity(nz, nz));


    // Innovation
    Eigen::VectorXd innovation = z - z_pred;
    if (use_full) {
      innovation(2) = wrapAngle(innovation(2));
    }


    // State update
    const Eigen::VectorXd correction = K * innovation;

    RCLCPP_INFO_STREAM_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "UKF mode=" << (use_full ? "FULL" : "IMU_ONLY")
        << " | innovation=" << innovation.transpose()
        << " | K_norm=" << K.norm()
        << " | correction_norm=" << correction.norm());

    x_hat_ = x_hat_ + correction;
    x_hat_(IDX_PSI) = wrapAngle(x_hat_(IDX_PSI));


    // Covariance update
    P_ = P_ - K * Pzz * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
  }

  /*
  ============================================================================
  Update continous heading for publishing, to avoid jumps in psi when wrapping around.
  This is only used for publishing, not for the filter itself.  
  ============================================================================
  */
 void updateContinuousPsi()
  {
    const double psi_wrapped = x_posterior_(IDX_PSI);
    double reference;
    if (!psi_publish_initialized_) {
      if(observer_cycle_.valid) {
        reference = observer_cycle_.psi;
      } else {
        reference = psi_wrapped;
      }
    }
    else {
        reference = psi_publis_continous_;
      }

    // if this cycle used a freash low-rate packet,  use the raw continous heading as the branch
    if (low_rate_update_this_cycle_ && observer_cycle_.valid)
    {
      reference = observer_cycle_.psi_raw;
    }
    psi_publis_continous_ = unwrapAngleNear(psi_wrapped, reference);
    psi_publish_initialized_ = true;
  }

  /*
  ============================================================================
  PUBLISH STATE
  ============================================================================
  We publish:
    1) The latest corrected state x_posterior_ as Float64MultiArray
    2) A compact custom StateObserver message
  */
  void publishStateEstimate()
  {
    // Full state vector
    std_msgs::msg::Float64MultiArray full_msg;
    full_msg.data.resize(NX);
    for (int i = 0; i < NX; ++i) {
      full_msg.data[static_cast<std::size_t>(i)] = x_posterior_(i);
    }
    full_msg.data[static_cast<std::size_t>(IDX_PSI)] = psi_publis_continous_;
    publisher_full_state_->publish(full_msg);


    // Custom observer message
    asv_interfaces::msg::StateObserver obs_msg;
    obs_msg.header.stamp = now();
    obs_msg.header.frame_id = my_id_;


    obs_msg.point.x = x_posterior_(IDX_X);
    obs_msg.point.y = x_posterior_(IDX_Y);
    // obs_msg.point.z = x_posterior_(IDX_PSI);
    obs_msg.point.z = psi_publis_continous_;


    obs_msg.velocity.x = x_posterior_(IDX_U);
    obs_msg.velocity.y = x_posterior_(IDX_V);
    obs_msg.velocity.z = x_posterior_(IDX_R);


    obs_msg.disturbances.x = x_posterior_(IDX_SEU);
    obs_msg.disturbances.y = x_posterior_(IDX_SEV);
    obs_msg.disturbances.z = x_posterior_(IDX_SER);


    publisher_state_estimate_->publish(obs_msg);


    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Publishing state estimate: x=%.2f, y=%.2f, psi_continuous=%.2f, psi_wrapped=%.2f, r=%.2f",
      x_posterior_(IDX_X), x_posterior_(IDX_Y),
      psi_publis_continous_, x_posterior_(IDX_PSI), x_posterior_(IDX_R));
  }

  /*
  ============================================================================
  PUBLISH FULL-FUSION STATE (LOW-RATE TOPIC)
  ============================================================================
  Published only when the full low-rate packet was incorporated this cycle.
  Stamped with the IMU time of that cycle so each sample lines up exactly
  with the matching 100 Hz sample when comparing offline.
  */
  void publishLowRateStateEstimate(const ImuData & imu_local)
  {
    asv_interfaces::msg::StateObserver obs_msg;

    obs_msg.header.stamp = imu_local.stamp;
    obs_msg.header.frame_id = my_id_;

    obs_msg.point.x = x_posterior_(IDX_X);
    obs_msg.point.y = x_posterior_(IDX_Y);
    // obs_msg.point.z = x_posterior_(IDX_PSI);
    obs_msg.point.z = psi_publis_continous_;

    obs_msg.velocity.x = x_posterior_(IDX_U);
    obs_msg.velocity.y = x_posterior_(IDX_V);
    obs_msg.velocity.z = x_posterior_(IDX_R);

    obs_msg.disturbances.x = x_posterior_(IDX_SEU);
    obs_msg.disturbances.y = x_posterior_(IDX_SEV);
    obs_msg.disturbances.z = x_posterior_(IDX_SER);

    publisher_state_estimate_lowrate_->publish(obs_msg);
  }


private:
  /*
  ============================================================================
  PARAMETERS
  ============================================================================
  */
  std::string my_id_;


  double Ts_ms_   = 10.0;
  double dt_      = 0.01;
  double gravity_ = 9.81;


  std::vector<double> Xu_;
  std::vector<double> Xv_;
  std::vector<double> Xr_;


  // UKF tuning
  double alpha_  = 1e-3;
  double beta_   = 2.0;
  double kappa_  = -2.0;
  double lambda_ = 0.0;
  double gamma_  = 0.0;


  // UKF weights
  Eigen::VectorXd Wm_;
  Eigen::VectorXd Wc_;


  /*
  ============================================================================
  UKF INTERNAL STATE
  ============================================================================
  */
  Eigen::VectorXd x_hat_;        // current working state
  Eigen::VectorXd x_posterior_;  // latest corrected state
  Eigen::MatrixXd P_;            // covariance
  Eigen::MatrixXd P_init_;       // initial covariance
  Eigen::MatrixXd Q_;            // process noise covariance
  Eigen::MatrixXd R_full_;       // full measurement covariance
  double psi_publis_continous_ = 0.0;  // continuous psi for publishing, to avoid jumps
  bool psi_publish_initialized_ = false;
 
  /*
  ============================================================================
  FLAGS
  ============================================================================
  */
  bool armed_ = false;
  bool initialized_ = false;


  // True only when a new low-rate packet has arrived
  bool have_fresh_low_rate_ = false;


  // Snapshot flag used inside one IMU cycle
  bool low_rate_update_this_cycle_ = false;


  /*
  ============================================================================
  SHARED DATA
  ============================================================================
  */
  std::mutex data_mutex_;
  ObserverData observer_latest_;
  ObserverData observer_cycle_;


  /*
  ============================================================================
  ROS INTERFACES
  ============================================================================
  */
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr subscriber_state_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_observer_data_;


  rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_estimate_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_full_state_;
  rclcpp::Publisher<asv_interfaces::msg::StateObserver>::SharedPtr publisher_state_estimate_lowrate_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};


/*
===============================================================================
MAIN
===============================================================================
*/
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);


  auto node = std::make_shared<UnscentedKalmanFilter>();
  rclcpp::spin(node);


  rclcpp::shutdown();
  return 0;
}

