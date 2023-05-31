### Description

- This code is being developed in ROS2 Humble
- It is part of the projects developed by the research group Optimization and Control of Distributed Systems
- Working with Navio2, Raspberry pi 4 and Ardupilot

# ASV_Loyola_US_low_level

![](https://www.uloyola.es/templates/v6/images/isologo_loyola_principal.svg)

# H1 Package

## asv_comunication

This package is included the nodes in charge of communication between the different ships of the fleet through the Zigbee modules.

## asv_control

This package are included the nodes in charge of the control, such as the state observers and the trajectory controllers.

## asv_interfaces

This package includes the messages, services and personalized actions for the execution of the different control algorithms, message declarations should not be included in the other nodes

## asv_bringup

This package includes the launchers used for tests or high-level executions, it is important to review the dependencies before including the execution of nodes.




