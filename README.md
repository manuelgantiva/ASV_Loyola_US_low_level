# ASV_Loyola_US_low_level

![](https://www.uloyola.es/templates/v6/images/isologo_loyola_principal.svg)

### Description

- Developed using ROS2 Humble  
- Part of the research projects by the Optimization and Control of Distributed Systems group  
- Compatible with Navio2, Navigator, and other Ardupilot-based systems that support MAVLink  
- Includes a lightweight node-based simulator for computationally efficient validation using a path-following strategy


## Table of Contents

1. [Requirements](#requirements)  
   - [MAVROS](#mavros)  
   - [Experimental Requirements](#experimental-requirements)  
2. [Clone the Repository](#clone-the-repository)  
3. [Compilation](#compilation)  
4. [Launching Your Simulation](#launching-your-simulation)


## Requirements

Before getting started, make sure the following are installed on your system:

- [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)  
- [Python 3](https://www.python.org/downloads/)

As with any ROS2 project, this one depends on several additional packages. The following ROS2 dependencies are required:

- `rclcpp`  
- `rclpy`  
- `example_interfaces`  
- `mavros_msgs`  
- `rcl_interfaces`  
- `geometry_msgs`  
- `tf2`  
- `tf2_ros`  
- `sensor_msgs`  
- `nav_msgs`
- `xacro`

 
### MAVROS

One of the key dependencies of this project is the `mavros` package. It serves as a communication bridge between ROS2 and any autopilot that supports the MAVLink protocol, such as Ardupilot. Through MAVROS, this project can send commands to the vehicle (throttle, steering, mode changes, etc.) and receive telemetry data (position, orientation, speed, etc.).

You can find the official repository and documentation here: [MAVROS GitHub (ros2 branch)](https://github.com/mavlink/mavros/tree/ros2)

To install it:

```bash
sudo apt install ros-humble-mavros
```

Then install the GeographicLib datasets required by MAVROS:

```bash
ros2 run mavros install_geographiclib_datasets.sh

# Alternative:
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```
### Experimental Requirements

This section is only relevant for experimental validation, as it requires additional hardware-specific configurations. In particular, setting up the XBee communication modules and the IMU is necessary to ensure correct integration with the onboard system.

#### Peripheral Configuration

These commands will create udev rules to identify the XBee and IMU modules when connected, making it easier to link them to your system. If you don't plan to use these peripherals (e.g., desktop-only development or reading from rosbag files), you can skip this step. First, download the files `bind_device.sh`, `imu_usb.rules`, and `xbee_usb.rules` from this [repository](https://github.com/manuelgantiva/asv_UL_Docker/tree/main/docker) and place them in a folder named `rules`:

```bash
cd /rules
sudo chmod 777 bind_device.sh
sudo sh bind_device.sh
cd ..
```


#### XBee Python Library

This library is used to simplify the connection with XBee modules. Even if you don't plan to use them, it is recommended to install it to avoid dependency issues:

```bash
pip install digi-xbee
```
## Clone the Repository

Start by creating your ROS 2 workspace (if you don't already have one), and clone this repository into the `src` folder:

```bash
mkdir -p ASV/src
cd ASV/src
git clone -b beckermn https://github.com/manuelgantiva/ASV_Loyola_US_low_level.git .
```
This will download the project into your workspace and place the contents directly under `src`.

### Compilation

Once the repository is cloned, go to the root of the workspace and build the project using colcon:

```bash
cd ..
colcon build ----executor sequential
```

If you only want to compile a specific package, you can use the following command instead:

```bash
cd ..
colcon build --packages-select [package_name]
```

Replace `[package_name]` with the actual name of the package you want to build.


## Launching Your Simulation

This section provides an example of how to launch the simulator along with the control strategy. It also includes visualization tools like `rviz2` and `rqt`, which help evaluate the performance of the controller in real time.

To launch the simulation, use the following command:

```bash
ros2 launch asv_bringup simulator_test.launch.py my_id:=1
```

The `my_id` argument specifies the vehicle to simulate and can take one of the following values: `0`, `1`, `3`, or `4`. These correspond to the pre-configured vehicle setups in the project.

If you want to enable recording during the simulation (to generate `rosbag` files for later processing), use this alternative command:

```bash

ros2 launch asv_bringup simulator_test.launch.py my_id:=1 rec:=true
```

The recorded rosbag files can be processed and analyzed using the tools available in the following [repository](https://github.com/manuelgantiva/ASV_ROS2_to_Matlab).

## Citation

If you use content from this repository or refer to the linked chapter, please cite it as:

> Manuel Gantiva, Thalia Morel, Guillermo Bejarano, Pablo Millan, and Federico Peralta. 2025. *From Concept to Control: Development of an Advanced ASV Platform for Testing*. In Press Springer Nature, *Smart Water Quality Monitoring* (pp. xx–xx). Springer Nature.
