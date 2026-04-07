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

# Incluir los directorios de encabezados de gurubi_c++

set(GUROBI_ROOT /opt/gurobi/linux)
include_directories(${GUROBI_ROOT}/include)
link_directories(${GUROBI_ROOT}/lib)

add_executable(pwm_mapper src/pwm_mapper.cpp)
ament_target_dependencies(pwm_mapper rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(mux_llc src/mux_llc.cpp)
ament_target_dependencies(mux_llc rclcpp example_interfaces mavros_msgs asv_interfaces)

add_executable(asv_tf2_broadcaster src/asv_tf2_broadcaster.cpp)
ament_target_dependencies(asv_tf2_broadcaster rclcpp tf2 tf2_ros geometry_msgs asv_interfaces nav_msgs)

add_executable(ifac_llc src/ifac_llc.cpp)
ament_target_dependencies(ifac_llc rclcpp geometry_msgs mavros_msgs asv_interfaces)

add_executable(mpc_llc src/mpc_llc.cpp)
ament_target_dependencies(mpc_llc rclcpp geometry_msgs mavros_msgs asv_interfaces std_msgs)
## Linkear el ejecutable con las librerías de Gurobi
target_link_libraries(mpc_llc gurobi_c++ gurobi110)

add_executable(mpc_hlc src/mpc_hlc.cpp)
ament_target_dependencies(mpc_hlc rclcpp geometry_msgs mavros_msgs std_msgs asv_library)
# Linkear el ejecutable con las librerías de Gurobi
target_link_libraries(mpc_hlc gurobi_c++ gurobi110)

add_executable(wang_mlc src/wang_mlc.cpp)
ament_target_dependencies(wang_mlc rclcpp geometry_msgs mavros_msgs asv_interfaces std_msgs asv_library)

add_executable(morel_hlc src/morel_hlc.cpp)
ament_target_dependencies(morel_hlc rclcpp geometry_msgs mavros_msgs asv_interfaces std_msgs asv_library)

add_executable(mod_wang_mlc src/mod_wang_mlc.cpp)
ament_target_dependencies(mod_wang_mlc rclcpp geometry_msgs mavros_msgs asv_interfaces std_msgs asv_library)

add_executable(filter_hlc src/filter_hlc.cpp)
ament_target_dependencies(filter_hlc rclcpp std_msgs mavros_msgs geometry_msgs)

# Add the executable for the l_theta_smc_formation node
# This node is used to control the formation of the ASV using the l_theta_smc algorithm
add_executable(l_theta_smc_formation src/l_theta_smc_formation.cpp)
ament_target_dependencies(l_theta_smc_formation rclcpp geometry_msgs mavros_msgs asv_interfaces std_msgs asv_library Eigen3)


# Install Targets
install(TARGETS
  pwm_mapper
  mux_llc
  asv_tf2_broadcaster
  ifac_llc
  wang_mlc
  mpc_llc
  morel_hlc
  mod_wang_mlc
  mpc_hlc
  filter_hlc
  l_theta_smc_formation
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
