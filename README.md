## Welcome to Veysi ADIN & Chunwoo Kim's ROS2 EtherCAT package.

  This repository contains ROS2 and EtherCAT based control framework for medical robots, but this implementation can be used with any robotic system with some modifications.Contains EtherCAT real-time thread with priority of 98. Software consists of four components (ROS2 nodes) : 
  
• EtherCAT node: Responsible for EtherCAT communication between master and slaves and publishes acquired feedback from slaves under /slave_feedback topic name in 1 kHz frequency. Additionally, subscribes to /master_commands topic, published from the control node, /gui_data topic published from the GUI node, and /safety_info topic published from the safety node. Sends control commands to slaves via EtherCAT communication using control node parameters. 


• Control node: Kinematic calculations will be done in this node. This node subscribes /slave_feedback topic published from the EtherCAT node and publishes control commands under the /master_commands topic. Currently this node retrieves button and joystick data from Xbox Controller via USB communication.
  
• GUI node: Consists of camera viewer and slave feedback visualizers such as motor state, communication state, and emergency button state. Publishes under /gui_buttons data consists of soft button events, subscribes to /master_ commands, and /slave_feedback, and /safety info topics to give visual feedback to the user. Current version of the GUI node shown in Figure 25. Control user interface buttons activated based on lifecycle node state.

• Safety node: Subscribes to all published topics and checks the consistency in the system, decides about the state of the system. System-related safety information is checked every cycle. Manages lifecycle node and state transitions are triggered from this node.
Published and subscribed topics for each node are shown in Table 3, and the details structure of the topics is shown in Table 4. Note that published topic messages can be changed by changing msg file contents in case custom message is required.

 Please check guides, links and documentations before installation, or using this control framework.

## Guides

- [ROS2](https://docs.ros.org/en/foxy/index.html)
- [ROS2 Life-cycle Node](https://design.ros2.org/articles/node_lifecycle.html)
- [ROS2 DDS](https://design.ros2.org/articles/ros_on_dds.html)
- [ROS2 QoS](https://design.ros2.org/articles/qos_deadline_liveliness_lifespan.html)
- [EtherCAT](https://www.ethercat.org/en/technology.html)
- [Etherlab Webpage](https://www.etherlab.org/en/ethercat/index.php)
- [IgH EtherCAT Library Documentation](https://www.etherlab.org/download/ethercat/ethercat-1.5.2.pdf)
- [Real-time Linux](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/start)
- [ROS2 Real-time Background](https://design.ros2.org/articles/realtime_background.html)
- [Article on EtherCAT-RT PREEMPT- Xenomai](https://www.ripublication.com/ijaer17/ijaerv12n21_94.pdf)

## Prerequisites
- [RT_Preempt Linux and IgH EtherCAT Implementation](https://github.com/veysiadn/IgHEtherCATImplementation)
- If you want to use [Xenomai-Instalattion](https://github.com/veysiadn/xenomai-install)
- [ROS2 Foxy Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [Dynamixel-SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK)

## Implementation
##### Don't forget to change the launch file, based on your system, custom node, CPU isolation etc.
  
```sh
git clone https://github.com/veysiadn/spinerobot_ws 
cd spinerobot_ws 
git checkout development
sudo -s
source /opt/ros/foxy/setup.bash
colcon build --symlink install
. install/setup.bash
ros2 launch launch_all_nodes.py
```

## Getting Started
### STEP 1 : 
  You should start your customization from ecat_globals.hpp file. In that file you can specify : 
  - Number of connected slaves and servo drives,
  - Control operation mode : Velocity Mode, Position Mode, Torque Mode, Cyclic Synchronous Velocity,Position and Torque modes are supported. 
  - Control Frequency
  - Enable/Disable Distributed Clock
  - Motors encoder resolution : note that if you are using different type of motors you might need to create different definitions for each motor.
  - Motor Gear Ratio :  note that if you are using different type of motors you might need to create different definitions for each motor.
  - Custom Slave : If you have different slave than the CiA402 supported servo drive you will need to define custom slave and PDO mapping for that custom slave.
  - Keep in mind that this software addresses connected slaves based on physical position with respect to the master. For example the 0th slave will be the first slave that is connected to your Ethernet port. 
  - If you have a custom slave, it must be in the end of slave chain.
  - 
### STEP 2 : 
  Once you did your initial configuration in the ecat_globals.hpp file. You can modify user input method in main.cpp file.
  - Currently this software uses XboxController buttons and joystick data to send control commands to the connected motors, if you want to use different input you can remove Xbox related control parameters and add your own.
  - If you want to use Xbox Controller for testing, keep in mind that each axis in the controller generates analog data in the range of -32768 ~ 32768.

### STEP 3 : 
  - Change configuration parameters for your motor in [SetConfigurationParameters()](https://veysiadn.github.io/ecat_userspace/classEthercatLifeCycleNode_1_1EthercatLifeCycle.html#a620253b4fe34f13f06a60f1fb12a81cc) function in ecat_lifecycle.cpp file, based on your operation mode. 
  You can check example page for detailed instructions on custom modifications, in Examples tab.
  - Change control parameters based on your selected operation mode in ecat_lifecycle.cpp file in the Update functions. For example if you want to use velocity mode in your control loop, you can change function content of UpdateVelocityModeParameters();

Once you did those changes you will need recompile the software using make and you can test the executable. 

### NOTE 
  - This software heavily tested on Maxon EPOS drivers, therefore if you want to use different servo driver you will need to check PDO mapping of your slave, or you can do custom PDO mapping by using MapCustomPdo function defined in ecat_node.cpp file. 

## Good Luck ⚡
