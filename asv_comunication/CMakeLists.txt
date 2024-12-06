cmake_minimum_required(VERSION 3.8)
project(asv_comunication)

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
find_package(rosbag2_cpp REQUIRED)
find_package(example_interfaces REQUIRED) # msgs and srv to basis comunication
find_package(std_msgs REQUIRED)           # msgs and srv to basis comunication
find_package(mavros_msgs REQUIRED)        # msgs to comunicate with mavros
find_package(rcl_interfaces REQUIRED)     # interface to use param with aardupilot and mavors
find_package(sensor_msgs REQUIRED)        # interface to use navio2's sensors
find_package(tf2 REQUIRED)        # interface to use navio2's sensors
find_package(tf2_ros REQUIRED)        # interface to use navio2's sensors
find_package(nav_msgs REQUIRED)        # interface to use navio2's sensors


include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

add_executable(testnode src/testnode.cpp)
ament_target_dependencies(testnode rclcpp std_msgs)

add_executable(bag_record src/bag_record.cpp)
ament_target_dependencies(bag_record rclcpp rosbag2_cpp sensor_msgs mavros_msgs asv_interfaces nav_msgs)

add_executable(rc_handler src/rc_handler.cpp)
ament_target_dependencies(rc_handler rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(ref_llc src/ref_llc.cpp)
ament_target_dependencies(ref_llc rclcpp std_msgs mavros_msgs geometry_msgs)

add_executable(ref_mlc src/ref_mlc.cpp)
ament_target_dependencies(ref_mlc rclcpp std_msgs mavros_msgs)

add_executable(ref_hlc src/ref_hlc.cpp)
ament_target_dependencies(ref_hlc rclcpp std_msgs mavros_msgs)

add_executable(apm_llc src/apm_llc.cpp)
ament_target_dependencies(apm_llc rclcpp geometry_msgs mavros_msgs asv_interfaces)

add_executable(imu_fix src/imu_fix.cpp)
ament_target_dependencies(imu_fix rclcpp std_msgs geometry_msgs sensor_msgs mavros_msgs)

# Install Targets
install(TARGETS
  testnode
  bag_record
  rc_handler
  ref_llc
  ref_mlc
  ref_hlc
  apm_llc
  imu_fix
DESTINATION lib/${PROJECT_NAME})

# Install Scripts python
ament_python_install_package(scripts)

install(PROGRAMS
  scripts/transceiver_xbee.py
  scripts/xbee_master.py
  scripts/xbee_slave.py
  scripts/imu_driver.py 
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
