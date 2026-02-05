# Contributing to ASV Loyola US low level

Thank you for your interest in contributing to the ASV Loyola project! While our physical platforms are located at Universidad Loyola Andalucía, we foster a global research community through our simulation and software ecosystem.

## How You Can Contribute

Since remote access to the physical ASVs is limited, we encourage the following types of remote contributions:

* Algorithm Development: Implementing new trajectory tracking, dynamic control, or state estimation nodes.
* Simulation Improvements: Enhancements to the asv_simulator or updates to the URDF models.
* Docker and Infrastructure: Optimizing the deployment environment or updating dependencies.
* Bug Reports: Identifying and fixing issues within the ROS 2 packages.

## Getting Started

1. Fork the Repository: Create your own copy of the main repository ([https://github.com/manuelgantiva/ASV_Loyola_US_low_level](https://github.com/manuelgantiva/ASV_Loyola_US_low_level)).
2. Setup the Environment: We strongly recommend using our Docker image ([https://github.com/manuelgantiva/asv_UL_Docker](https://github.com/manuelgantiva/asv_UL_Docker)) to ensure all dependencies (ROS 2 Humble, Ubuntu 22.04, etc.) are correctly configured.
3. Select the Correct Branch: Always use the benchmark branch as your base for new features or fixes.

## Development Workflow

1. Branching: Create a feature branch from benchmark:
git checkout -b feature/your-contribution-name
2. Build and Test: Ensure the code compiles using "colcon build".
3. Simulation Validation: All navigation or control logic must be validated in the asv_simulator before submission. Provide evidence (e.g., plots or rosbag summaries) of the performance in the virtual environment.

## Pull Request Guidelines

When submitting a Pull Request (PR):

* Clear Description: Explain the "what" and the "why" of your changes.
* Documentation: Update relevant README files or comments if you change how a node or parameter works.
* Standards: Follow standard ROS 2 coding conventions for Python (rclpy) and C++ (rclcpp).

## Experimental Validation

If your contribution demonstrates significant improvements or novel capabilities in simulation, our research team may consider it for validation on the physical ASV platforms during our scheduled experimental trials at Universidad Loyola.

## Contact

For questions regarding the ODS research group or specific inquiries about experimental trials, please contact the group coordinators via the links provided in the main README.
