cmake_minimum_required(VERSION 3.8)
project(asv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)      # msgs and srv to teleop_key and Local gps
find_package(asv_interfaces REQUIRED)
find_package(example_interfaces REQUIRED) # msgs and srv to basis comunication
find_package(std_msgs REQUIRED)           # msgs and srv to basis comunication
find_package(mavros_msgs REQUIRED)        # msgs to comunicate with mavros
find_package(rcl_interfaces REQUIRED)     # interface to use param with aardupilot and mavors
find_package(sensor_msgs REQUIRED)        # interface to use navio2's sensors
find_package(tf2 REQUIRED)        # interface to use navio2's sensors
find_package(tf2_ros REQUIRED)        # interface to use navio2's sensors

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})


add_executable(testnode src/testnode.cpp)
ament_target_dependencies(testnode rclcpp std_msgs)

add_executable(change_mode src/change_mode.cpp)
ament_target_dependencies(change_mode rclcpp example_interfaces mavros_msgs)

add_executable(listener_rc src/listener_rc.cpp)
ament_target_dependencies(listener_rc rclcpp example_interfaces mavros_msgs)

add_executable(pwm_mapper src/pwm_mapper.cpp)
ament_target_dependencies(pwm_mapper rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(pwm_key src/pwm_key.cpp)
ament_target_dependencies(pwm_key rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(observer src/observer.cpp)
ament_target_dependencies(observer rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces tf2 tf2_ros geometry_msgs)

add_executable(observer_liu src/observer_liu.cpp)
ament_target_dependencies(observer_liu rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces tf2 tf2_ros geometry_msgs)

add_executable(observer_zono src/observer_zono.cpp)
ament_target_dependencies(observer_zono rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces geometry_msgs)

add_executable(asv_tf2_broadcaster src/asv_tf2_broadcaster.cpp)
ament_target_dependencies(asv_tf2_broadcaster rclcpp tf2 tf2_ros geometry_msgs)

# Install Targets
install(TARGETS
  testnode
  change_mode
  listener_rc
  pwm_mapper
  pwm_key
  observer
  observer_liu
  observer_zono
  asv_tf2_broadcaster
DESTINATION lib/${PROJECT_NAME})

# Install Scripts python
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/testnode.py 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
