#include "rclcpp/rclcpp.hpp"                    // ROS2 C++ API
#include "sensor_msgs/msg/imu.hpp"              // IMU message type
#include "mavros_msgs/msg/state.hpp"            // Armed/disarmed state
#include "std_msgs/msg/float32_multi_array.hpp" // observer/data_sensores
#include "std_msgs/msg/float64_multi_array.hpp" // UKF output state vector
#include "asv_interfaces/msg/state_observer.hpp" // Custom state observer message


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
      rclcpp::rclcpp::SensorDataQoS());


    // Full estimated state vector
    publisher_full_state_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/" + my_id_ + "/observer/state_ukf",
      rclcpp::SensorDataQoS());


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
    double psi = 0.0;
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
    observer_latest_.psi         = wrapAngle(static_cast<double>(msg->data[2]));
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
      Xv_.at(1) * v * std::abs(r) +
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


    const double ax_model = Gm(0) + sigma_m(0) + x(IDX_SEU) - x(IDX_R) * x(IDX_V) + x(IDX_BAX);
    const double ay_model = Gm(1) + sigma_m(1) + x(IDX_SEV) + x(IDX_R) * x(IDX_U) + x(IDX_BAY);
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
    P_pred += Q_;


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
      R_use = R_full_;
    } else {
      // IMU-only measurement noise for [ax ay r]
      R_use = R_full_.block(3, 3, 3, 3);
    }


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
    x_hat_ = x_hat_ + K * innovation;
    x_hat_(IDX_PSI) = wrapAngle(x_hat_(IDX_PSI));


    // Covariance update
    P_ = P_ - K * Pzz * K.transpose();
    P_ = 0.5 * (P_ + P_.transpose());
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
    publisher_full_state_->publish(full_msg);


    // Custom observer message
    asv_interfaces::msg::StateObserver obs_msg;
    obs_msg.header.stamp = now();
    obs_msg.header.frame_id = my_id_;


    obs_msg.point.x = x_posterior_(IDX_X);
    obs_msg.point.y = x_posterior_(IDX_Y);
    obs_msg.point.z = x_posterior_(IDX_PSI);


    obs_msg.velocity.x = x_posterior_(IDX_U);
    obs_msg.velocity.y = x_posterior_(IDX_V);
    obs_msg.velocity.z = x_posterior_(IDX_R);


    obs_msg.disturbances.x = x_posterior_(IDX_SEU);
    obs_msg.disturbances.y = x_posterior_(IDX_SEV);
    obs_msg.disturbances.z = x_posterior_(IDX_SER);


    publisher_state_estimate_->publish(obs_msg);


    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Publishing state estimate: x=%.2f, y=%.2f, psi=%.2f, r=%.2f",
      x_posterior_(IDX_X), x_posterior_(IDX_Y),
      x_posterior_(IDX_PSI), x_posterior_(IDX_R));
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

