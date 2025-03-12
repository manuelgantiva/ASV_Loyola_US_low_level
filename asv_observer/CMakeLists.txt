cmake_minimum_required(VERSION 3.8)
project(asv_observer)

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
find_package(asv_library REQUIRED)
find_package(example_interfaces REQUIRED) # msgs and srv to basis comunication
find_package(std_msgs REQUIRED)           # msgs and srv to basis comunication
find_package(mavros_msgs REQUIRED)        # msgs to comunicate with mavros
find_package(rcl_interfaces REQUIRED)     # interface to use param with aardupilot and mavors
find_package(sensor_msgs REQUIRED)        # interface to use navio2's sensors
find_package(tf2 REQUIRED)        # interface to use navio2's sensors
find_package(tf2_ros REQUIRED)        # interface to use navio2's sensors
find_package(nav_msgs REQUIRED)        # interface to use navio2's sensors

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_executable(mux_obs src/mux_obs.cpp)
ament_target_dependencies(mux_obs rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(observer_guille src/observer_guille.cpp)
ament_target_dependencies(observer_guille rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces tf2 tf2_ros geometry_msgs nav_msgs)

add_executable(observer_liu src/observer_liu.cpp)
ament_target_dependencies(observer_liu rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces tf2 tf2_ros geometry_msgs nav_msgs)

add_executable(observer_zono src/observer_zono.cpp)
ament_target_dependencies(observer_zono rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces asv_library tf2 tf2_ros geometry_msgs nav_msgs)

add_executable(observer_param src/observer_param.cpp)
ament_target_dependencies(observer_param rclcpp std_msgs sensor_msgs mavros_msgs asv_interfaces geometry_msgs nav_msgs)

# Install Targets
install(TARGETS
  mux_obs
  observer_guille
  observer_liu
  observer_zono
  observer_param
DESTINATION lib/${PROJECT_NAME})

# Install Scripts python
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/testnode.py 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
