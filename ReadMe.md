For simplified Chinese version: [简体中文版](./ReadMe_cn.md)    
For **UFACTORY Lite 6/850** users, make sure you have followed the instructions before chapter 4.7 on this page, then switch to [ReadMe for Lite6/UF850](./ReadMe_others.md).    
For **kinetic** users, please use the [kinetic branch](https://github.com/xArm-Developer/xarm_ros/tree/kinetic).

## Important Notice:
&ensp;&ensp;Topic "**xarm_cgpio_states**" has been renamed to "**controller_gpio_states**".  

&ensp;&ensp;After using xArm C++ SDK as sub-module, the use of **/xarm/set_tool_modbus** service has been modified, compared with old version, the redundant '***0x09***' byte in response data has been ***removed***！  
&ensp;&ensp;Due to robot communication data format change, ***early users*** (xArm shipped ***before June 2019***) are encouraged to ***upgrade*** their controller firmware immediately to drive the robot normally in future updates as well as to use newly developed functions. Please contact our staff to get instructions of the upgrade process. The old version robot driver can still be available in ***'legacy'*** branch, however, it will not be updated any more.   

&ensp;&ensp;You MUST follow **chapter 3** to install additional packages needed before any usage of xarm_ros packages. Otherwise, unexpected errors may occur.

&ensp;&ensp;If developing with **Moveit**, it is highly recommended to use **DIRECT network cable connection** between controller box and your PC, and no intermediate switches or routers, or the communication latency may have a bad impact on trajectory execution.  

&ensp;&ensp; When updating this package, please remember to [check the submodule update](#421-update-the-package) as well!  

# Contents:  
* [1. Introduction](#1-introduction)
* [2. Update History](#2-update-summary)
* [3. Preparations (**MUST DO**)](#3-preparations-before-using-this-package)
* [4. Get Started](#4-getting-started-with-xarm_ros)
* [5. Package Description & Usage Guidance](#5-package-description--usage-guidance)
    * [5.1 xarm_description](#51-xarm_description)  
    * [5.2 xarm_gazebo](#52-xarm_gazebo)  
    * [5.3 xarm_controller](#53-xarm_controller)  
    * [5.4 xarm_bringup](#54-xarm_bringup)  
    * [5.5 ***xarm7_moveit_config***](#55-xarm7_moveit_config)  
        * [5.5.1 Add Custom Tool Model For Moveit](#551-add-custom-tool-model-for-moveit)  
    * [5.6 ***xarm_planner***](#56-xarm_planner)  
    * [5.7 ***xarm_api/xarm_msgs (Online Planning Modes Added)***](#57-xarm_apixarm_msgs)  
        * [5.7.1 Starting xArm by ROS service (***priority for the following operations***)](#starting-xarm-by-ros-service)  
        * [5.7.2 Joint space or Cartesian space command example](#joint-space-or-cartesian-space-command-example)
        * [5.7.3 Tool/Controller I/O Operations](#tool-io-operations)  
        * [5.7.4 Getting status feedback](#getting-status-feedback)  
        * [5.7.5 Setting Tool Center Point Offset](#setting-tool-center-point-offset)  
        * [5.7.6 Clearing Errors](#clearing-errors)  
        * [5.7.7 Gripper Control](#gripper-control)
        * [5.7.8 Vacuum Gripper Control](#vacuum-gripper-control)
        * [5.7.9 Tool Modbus communication](#tool-modbus-communication)
        * [5.7.10 'report_type' argument](#report_type-argument)
    * [5.8 ***xarm_moveit_servo***](#58-xarm_moveit_servo)
* [6. Mode Change(***Updated***)](#6-mode-change)
    * [6.1 Mode Explanation](#61-mode-explanation)
    * [6.2 Proper way to change modes](#62-proper-way-to-change-modes)
* [7. xArm Vision](#7-xarm-vision)
    * [7.1 Installation of dependent packages](#71-installation-of-dependent-packages)
    * [7.2 Hand-eye Calibration Demo](#72-hand-eye-calibration-demo)
    * [7.3 Vision Guided Grasping Demo](#73-vision-guided-grasping-demo)
    * [7.4 Adding RealSense D435i model to simulated xArm](#74-adding-realsense-d435i-model-to-simulated-xarm)
    * [7.5 Color Cube Grasping Demo (Simulation + Real Hardware)](#75-color-cube-grasping-demo)
* [8. Other Examples](#8-other-examples)
    * [8.0 An example of demonstrating redundancy resolution using MoveIt](https://github.com/xArm-Developer/xarm_ros/tree/master/examples/xarm7_redundancy_res)
    * [8.1 Multi-xArm5 (separate control)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#1-multi_xarm5-controlled-separately)
    * [8.2 Servo_Cartesian](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)
    * [8.3 Servo_Joint](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory)
    * [8.4 Dual xArm6 controlled with one moveGroup node](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#4-dual-xarm6-controlled-with-one-movegroup-node)
    * [8.5 Record and playback trajectories](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#5-run-recorded-trajectory-beta)
    * [8.6 Online target update for dynamic following task(**NEW**)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)

# 1. Introduction
   &ensp;&ensp;This repository contains the 3D models of xArm series and demo packages for ROS development and simulations.Developing and testing environment: Ubuntu 16.04/18.04/20.04 + ROS Kinetic/Melodic/Noetic.  
   ***Instructions below is based on xArm7, other model user can replace 'xarm7' with 'xarm6' or 'xarm5' where applicable.***

# 2. Update Summary
   This package is still under development and improvement, tests, bug fixes and new functions are to be updated regularly in the future. 
   * Add xArm 7 description files, meshes and sample controller demos for ROS simulation and visualization.
   * Add Moveit! planner support to control Gazebo virtual model and real xArm, but the two can not launch together.
   * Add Direct control of real xArm through Moveit GUI, please use it with special care.
   * Add xArm hardware interface to use ROS position_controllers/JointTrajectoryController on real robot.
   * Add xArm 6 and xArm 5 simulation/real robot control support.
   * Add simulation model of xArm Gripper.
   * Add demo to control dual xArm6 through Moveit.
   * Add xArm Gripper action control.
   * Add xArm-with-gripper Moveit development packages.
   * Add vacuum gripper model and xArm-with-vacuum-gripper Moveit development packages (under /examples dir).
   * Thanks to [Microsoft IoT](https://github.com/ms-iot), xarm_ros can now be compiled and run on Windows platform.
   * Add velocity control mode for joint and Cartesian space. (**xArm controller firmware version >= 1.6.8** required)  
   * Add support for [custom tool model for Moveit](#551-add-custom-tool-model-for-moveit)  
   * Add timed-out version of velocity control mode, for better safety consideration. (**xArm controller firmware version >= 1.8.0** required)  
   * Add xArm Vision and RealSense D435i related demo. Migrate previous "xarm_device" into xarm_vision/camera_demo.
   * xarm_controler (xarm_hw) no longer uses the SDK through service and topic, but directly calls the SDK interface.
   * Add text interpretation for Controller Error code, returned from "get_err" service.
   * Support UFACTORY Lite 6 model. 
   * [Beta] Added two more torque-related topics (temporarily do not support third-party torque sensors): /xarm/uf_ftsensor_raw_states (raw data) and /xarm/uf_ftsensor_ext_states (filtered and compensated data)
   * (2022-09-07) Add service(__set_tgpio_modbus_timeout__/__getset_tgpio_modbus_data__), choose whether to transparently transmit Modbus data according to different parameters
   * (2022-09-07) Update submodule xarm-sdk to version 1.11.0
   * (2022-11-16) Add torque related services: /xarm/ft_sensor_enable, /xarm/ft_sensor_app_set, /xarm/ft_sensor_set_zero, /xarm/ft_sensor_cali_load, /xarm/get_ft_sensor_error
   * (2023-02-10) Added xarm_moveit_servo to support xbox controller/SpaceMouse/keyboard control
   * (2022-02-18) Automatically saving in service(/xarm/ft_sensor_cali_load) and add torque related service(/xarm/ft_sensor_iden_load)
   * (2023-02-27) Added service to control Lite6 Gripper(/ufactory/open_lite6_gripper, /ufactory/close_lite6_gripper, /ufactory/stop_lite6_gripper)(Note: Once stop, close will be invalid, you must open first to enable control)
   * (2023-03-29) Added the launch parameter model1300 (default is false), and replaced the model of the end of the xarm robot arm with the 1300 series
   * (2023-04-20) Update the URDF file, adapt to ROS1 and ROS2, and load the inertia parameters of the link from the configuration file according to the SN
   * (2023-04-20) Added launch parameter `add_realsense_d435i` (default is false), supports loading Realsense D435i model
   * (2023-04-20) Added the launch parameter `add_d435i_links` (default is false), which supports adding the link relationship between D435i cameras when loading the RealSense D435i model. It is only useful when `add_realsense_d435i` is true
   * (2023-04-20) Added the launch parameter `robot_sn`, supports loading the inertia parameters of the corresponding joint link, and automatically overrides the `model1300` parameters
   * (2023-04-20) Added launch parameters `attach_to`/`attach_xyz`/`attach_rpy` to support attaching the robot arm model to other models
   * (2023-04-21) Added [services usage documentation](xarm_api/ReadMe.md) in xarm_api
   * (2023-06-07) Added support for __UFACTORY850__ robotic arm
   * (2023-06-07) Added [uf_robot_moveit_config](uf_robot_moveit_config/Readme.md), support xArm/Lite6/UFACTORY850 series of robotic arm controls with moveit, which may replace these packages in the future. See instructions for [uf_robot_moveit_config](uf_robot_moveit_config/Readme.md)
      - xarm5_moveit_config
      - xarm5_gripper_moveit_config
      - xarm5_vacuum_moveit_config
      - xarm6_moveit_config
      - xarm6_gripper_moveit_config
      - xarm6_vacuum_moveit_config
      - xarm7_moveit_config
      - xarm7_gripper_moveit_config
      - xarm7_vacuum_moveit_config
      - lite6_moveit_config
   * (2023-10-12) Added the generation and use of joint kinematics parameter files (only supports __uf_robot_moveit_config__, see the description of the general parameter `kinematics_suffix` in [uf_robot_moveit_config](uf_robot_moveit_config/Readme.md))

# 3. Preparations before using this package

## 3.1 Install dependent package module
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> (if use Gazebo)   
   ros_control: <http://wiki.ros.org/ros_control> (remember to select your correct ROS distribution)  
   moveit_core: <https://moveit.ros.org/install/>  
   
## 3.2 Go through the official tutorial documents
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 3.3 Download the 'table' 3D model
&ensp;&ensp;In Gazebo simulator, navigate through the model database for 'table' item, drag and place the 3D model inside the virtual environment. It will then be downloaded locally, as 'table' is needed for running the demo.

## 3.4 Install "mimic_joint_plugin" for xArm Gripper simulation in Gazebo
&ensp;&ensp;If simulating xArm Gripper in Gazebo is needed, [**mimic_joint_plugin**](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins) by courtesy of Konstantinos Chatzilygeroudis (@costashatz) needs to be installed in order to make the mimic joints behave normally in Gazebo. Usage of this plugin is inspired by [this tutorial](https://github.com/mintar/mimic_joint_gazebo_tutorial) from @mintar.   

12/22/2020: Refer to issue #53, Please Note this plugin has recently been **deprecated**, if you plan to use [new version](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), please change "libroboticsgroup_gazebo_mimic_joint_plugin.so" to "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so" in files xarm_ros/xarm_gripper/urdf/xarm_gripper.gazebo.xacro and xarm_ros/xarm_description/urdf/gripper/xarm_gripper.gazebo.xacro. 

# 4. Getting started with 'xarm_ros'
   
## 4.1 Create a catkin workspace. 
   &ensp;&ensp;If you already have a workspace, skip and move on to next part.
   Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 4.2 Obtain the package
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git --recursive
   ```

## 4.2.1 update the package
   ```bash
   $ cd ~/catkin_ws/src/xarm_ros
   
   # If you did not use the --recursive or --recurse-submodules option when cloning, use this command to initialize and update all submodules
   $ git submodule update --init --recursive
   
   # Pull the main repository and update the submodule
   $ git pull --recurse-submodules
   ```

## 4.3 Install other dependent packages:
   ```bash
   $ rosdep update
   $ rosdep check --from-paths . --ignore-src --rosdistro kinetic
   ```
   Please change 'kinetic' to the ROS distribution you use. If there are any missing dependencies listed. Run the following command to install:  
   ```bash
   $ rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
   ```
   And change 'kinetic' to the ROS distribution you use.  

## 4.4 Build the code
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.5 Source the setup script
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
Skip above operation if you already have that inside your ~/.bashrc. Then do:
```bash
$ source ~/.bashrc
```
## 4.6 First try out in RViz:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```

## 4.7 Run the demo in Gazebo simulator
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true] [add_gripper:=true] [add_vacuum_gripper:=true] 
   ```
&ensp;&ensp;Add the "run_demo" option if you wish to see a pre-programed loop motion in action. The command trajectory is written in xarm_controller\src\sample_motion.cpp. And the trajectory in this demo is controlled by pure position interface.   
&ensp;&ensp;Add the "add_gripper" option if you want to see the xArm Gripper attached at the tool end.  
&ensp;&ensp;Add the "add_vacuum_gripper" option if you want to see the xArm Vacuum Gripper attached at the tool end. Please note ONLY ONE end effector can be attached.  

# 5. Package description & Usage Guidance
   
## 5.1 xarm_description
   &ensp;&ensp;xArm description files, mesh files and gazebo plugin configurations, etc. It's not recommended to change the xarm description file since other packages depend on it. 

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world description files and simulation launch files. User can add or build their own models in the simulation world file.

## 5.3 xarm_controller
   &ensp;&ensp;Controller configurations, hardware_interface, robot command executable source, scripts and launch files. User can deploy their program inside this package or create their own. ***Note that*** effort controllers defined in xarm_controller/config are just examples for simulation purpose, when controlling the real arm, only 'position_controllers/JointTrajectoryController' interface is provided. User can add their self-defined controllers as well, refer to: http://wiki.ros.org/ros_control (controllers)

## 5.4 xarm_bringup  
&ensp;&ensp;launch files to load xarm driver to enable direct control of real xArm hardware.  

## 5.5 xarm7_moveit_config
Please note: xarm_moveit_config related packages will limit all joints within `[-pi, pi]`, it seems that moveit tend to generate plans involving greater joint motions if not limited within this range. This limit can be canceled by setting "limited:=false" in `...moveit_config/launch/planning_context.launch`.   

For any model which needs **kinematic calibration correction** added to the URDF, please check [uf_robot_moveit_config](uf_robot_moveit_config/Readme.md), this is a **unified** `moveit_config` package for all our models and supports new features.   

&ensp;&ensp;This package is partially generated by moveit_setup_assistant, could use with Moveit Planner and Rviz visualization. If you have Moveit! installed, you can try the demo. 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```
#### To run Moveit! motion planner along with Gazebo simulator:  
   1. If no xArm gripper needed, first run:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   Then in another terminal:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   2. If **xArm gripper needs to be attached**, first run:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch add_gripper:=true
   ```
   Then in another terminal:
   ```bash
   $ roslaunch xarm7_gripper_moveit_config xarm7_gripper_moveit_gazebo.launch
   ```
   If you have a satisfied motion planned in Moveit!, hit the "Execute" button and the virtual arm in Gazebo will execute the trajectory.  

   3. If **xArm vacuum gripper needs to be attached**, just replace "gripper" with "vacuum_gripper" in above gripper example.  

#### To run Moveit! motion planner to control the real xArm:  
   First make sure the xArm and the controller box are powered on, then execute:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address> [velocity_control:=false] [report_type:=normal]
   ```
   Examine the terminal output and see if any error occured during the launch. If not, just play with the robot in Rviz and you can execute the sucessfully planned trajectory on real arm. But be sure it will not hit any surroundings before execution!   

   `velocity_control` is optional, if set to `true`, velocity controller and velocity interface will be used rather than position control. `report_type` is also optional, refer [here](#report_type-argument).  

#### To run Moveit! motion planner to control the real xArm with xArm Gripper attached:  
   First make sure the xArm and the controller box are powered on, then execute:  
   ```bash
   $ roslaunch xarm7_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   It is better to use this package with real xArm gripper, since Moveit planner will take the gripper into account for collision detection.  

#### To run Moveit! motion planner to control the real xArm with xArm Vacuum Gripper attached:  
   First make sure the xArm and the controller box are powered on, then execute:  
   ```bash
   $ roslaunch xarm7_vacuum_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   It is better to use this package with real xArm vacuum gripper, since Moveit planner will take the vacuum gripper into account for collision detection.  

## 5.5.1 Add custom tool model for Moveit
&ensp;&ensp;***This part may require ROS Melodic or later versions to function well***  
&ensp;&ensp;For __xarm5_moveit_config__/__xarm6_moveit_config__/__xarm7_moveit_config__, customized tool models maybe added to the tool flange through quick-configuration parameters listed below，thus to enable Tool offset and 3D collision checking during Moveit motion planning. (Notice：configuration through '/xarm/set_tcp_offset' service will not be effective in Moveit planning!) 

### Examples:
   ```bash
   # attaching box model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=box

   # attaching cylinder model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=cylinder

   # attaching sphere model:
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=sphere

   # attaching customized mesh model:（Here take xarm vacuum_gripper as an example，if the mesh model could be placed in: 'xarm_description/meshes/other'directory，'geometry_mesh_filename' argument can be simplified to be just the filename）
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=mesh geometry_mesh_filename:=package://xarm_description/meshes/vacuum_gripper/xarm/visual/vacuum_gripper.stl geometry_mesh_tcp_xyz:='"0 0 0.126"'
   ```

### Argument explanations:
- __add_other_geometry__: default to be false，indicating whether to add other geometry model to the tool.
- __geometry_type__: geometry shapes to be added，as one of 'box/cylinder/sphere/mesh', there are different parameters required for different types.  
- __geometry_height__: height of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box/cylinder/sphere.
- __geometry_radius__: radius of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: cylinder/sphere.
- __geometry_length__: length of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box.
- __geometry_width__: width of geometry shape，unit: meter，default value: 0.1，effective for geometry_type: box.
- __geometry_mesh_filename__: geometry shape，effective for geometry_type: mesh.
- __geometry_mesh_origin_xyz__: position offset from mesh base coordinate to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_origin_rpy__: orientation offset from mesh base coordinate to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_tcp_xyz__: the positional TCP offset with respect to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.
- __geometry_mesh_tcp_rpy__: the orientational TCP offset with respect to xarm tool-flange coordinate, default: "0 0 0"，effective for geometry_type: mesh.  


## 5.6 xarm_planner:
&ensp;&ensp;This implemented simple planner interface is based on move_group from Moveit! and provide ros service for users to do planning & execution based on the requested target, user can find detailed instructions on how to use it inside [*xarm_planner package*](./xarm_planner/ReadMe.md).  
#### To launch the xarm simple motion planner together with the real xArm:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7|6|5> add_(vacuum_)gripper:=<true|false>
```
Argument 'robot_dof' specifies the number of joints of your xArm (default is 7). Now xarm_planner supports model with gripper or vacuum_gripper attached. Please specify "**add_gripper**" or "**add_vacuum_gripper**" argument if needed.    

## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;These two packages provide user with the ros service wrapper of the functions in xArm SDK. There are 12 types of motion command (service names) supported，please set correct robot mode first, refer to [mode change section](#6-mode-change):  
#### Robot Mode 0:
* <font color=blue>[move_joint](#1-joint-space-motion):</font> joint space point to point command, given target joint angles, max joint velocity and acceleration. Corresponding function in SDK is "set_servo_angle()".  
* <font color=blue>[move_line](#2-cartesian-space-motion-in-base-coordinate):</font> straight-line motion to the specified Cartesian Tool Centre Point(TCP) target. Corresponding function in SDK is "set_position()"[blending radius not specified].  
* <font color=blue>move_lineb:</font> straight-line motion, and blending continuously with next motion. Normally works in the form of a list of known via points followed by target Cartesian point. Each motion segment is straight-line with Arc blending at the via points, to make velocity continuous. Corresponding function in SDK is "set_position()"[wait=false and blending radius specified]. Please refer to [move_test.cpp](./xarm_api/test/move_test.cpp) and [blended_motion_test.py](./xarm_api/scripts/blended_motion_test.py) for example code, `/xarm/wait_for_finish` parameter has to be `false` for successful blending calculation.   
* <font color=blue>move_jointb:</font> joint space point to point motion, and blending continuously with next motion. It can be used together with "move_lineb" for joint-linear blending motions, as long as the via points are known, and blending radius is properly specified, velocity will be continuous during the execution. Corresponding function in SDK is "set_servo_angle()"[wait=false and blending radius specified]. Please refer to [blended_motion_test.py](./xarm_api/scripts/blended_motion_test.py) for example code. `/xarm/wait_for_finish` parameter has to be `false` for successful blending calculation.   
* <font color=blue>[move_line_tool](#3-cartesian-space-motion-in-tool-coordinate):</font> straight-line motion based on the **Tool coordinate system** rather than the base system. Corresponding function in SDK is "set_tool_position()".  
Please ***keep in mind that*** before calling the 4 motion services above, first set robot mode to be 0, then set robot state to be 0, by calling relavent services. Meaning of the commands are consistent with the descriptions in product ***user manual***, other xarm API supported functions are also available as service call. Refer to [xarm_msgs package](./xarm_msgs/) for more details and usage guidance.  

* <font color=blue>[move_line_aa](#4-cartesian-space-motion-in-axis-angle-orientation):</font> straight-line motion, with orientation expressed in **Axis-angle** rather than roll-pitch-yaw angles. Please refer to xArm user manual for detailed explanation of axis-angle before using this command.   

#### Robot Mode 1:
* <font color=blue>[move_servo_cart](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)/[move_servoj](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory):</font> streamed high-frequency trajectory command execution in Cartesian space or joint space. Corresponding functions in SDK are set_servo_cartesian() and set_servo_angle_j(). An alternative way to implement <font color=red>velocity control</font>. Special **RISK ASSESMENT** is required before using them. Please read the guidance carefully at [examples chapter 2-3](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory).   

#### Robot Mode 4:
* <font color=blue>[velo_move_joint/velo_move_joint_timed](#5-joint-velocity-control):</font> Joint motion with specified velocity for each joint (unit: rad/s), with maximum joint acceleration configurable by `set_max_acc_joint` service.  

#### Robot Mode 5:
* <font color=blue>[velo_move_line/velo_move_line_timed](#6-cartesian-velocity-control):</font> Linear motion of TCP with specified velocity in mm/s (position) and rad/s (orientation in **axis-angular_velocity**), with maximum linear acceleration configurable by `set_max_acc_line` service.  

#### Robot Mode 6: (Firmware >= v1.10.0)
* <font color=blue>[move_joint](#1-joint-space-motion):</font> Online joint space replanning to the new joint angles, with new max joint velocity and acceleration. Joint velocities and accelerations are continuous during transition, however the velocity profiles may not be synchronous and the final reached positions may have small errors. **This function is mainly for dynamic response without self trajectory planning requirement like servo joint commands**. `/xarm/wait_for_finish` parameter has to be `false` for successful transition. Corresponding function in SDK is "set_servo_angle(wait=false)" under mode 6. [Instructions](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update) 

#### Robot Mode 7: (Firmware >= v1.11.0)
* <font color=blue>[move_line](#2-cartesian-space-motion-in-Base-coordinate):</font> Online Cartesian space replanning to the new target coordinate, with new max linear velocity and acceleration. Velocities and accelerations are continuous during transition, **This function is mainly for dynamic response without self trajectory planning requirement like servo cartesian commands**. `/xarm/wait_for_finish` parameter has to be `false` for successful transition. Corresponding function in SDK is "set_position(wait=false)" under mode 7. [Instructions](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)   

#### Starting xArm by ROS service:

&ensp;&ensp;First startup the service server for xarm7, ip address is just an example:  
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128 report_type:=normal
```
The argument `report_type` is explained [here](#report_type-argument).  

&ensp;&ensp;Then make sure all the servo motors are enabled, refer to [SetAxis.srv](/xarm_msgs/srv/SetAxis.srv):
```bash
$ rosservice call /xarm/motion_ctrl 8 1
```
&ensp;&ensp;Before any motion commands, set proper robot mode(0: POSE) and state(0: READY) ***in order***, refer to [SetInt16.srv](/xarm_msgs/srv/SetInt16.srv):    
```bash
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```

#### Joint space or Cartesian space command example:
&ensp;&ensp;Please note that all the angles must use the unit of ***radian***. All motion commands use the same type of srv request: [Move.srv](./xarm_msgs/srv/Move.srv).   

##### 1. Joint space motion:
&ensp;&ensp;To call joint space motion with max speed 0.35 rad/s and acceleration 7 rad/s^2:   
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;To go back to home (all joints at 0 rad) position with max speed 0.35 rad/s and acceleration 7 rad/s^2:  
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```
##### 2. Cartesian space motion in Base coordinate:
&ensp;&ensp;To call Cartesian motion to the target expressed in robot BASE Coordinate, with max speed 200 mm/s and acceleration 2000 mm/s^2:
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
##### 3. Cartesian space motion in Tool coordinate:
&ensp;&ensp;To call Cartesian motion expressed in robot TOOL Coordinate, with max speed 200 mm/s and acceleration 2000 mm/s^2, the following will move a **relative motion** (delta_x=50mm, delta_y=100mm, delta_z=100mm) along the current Tool coordinate, no orientation change:
```bash
$ rosservice call /xarm/move_line_tool [50,100,100,0,0,0] 200 2000 0 0
```
##### 4. Cartesian space motion in Axis-angle orientation:
&ensp;&ensp;Corresponding service for Axis-angle motion is [MoveAxisAngle.srv](./xarm_msgs/srv/MoveAxisAngle.srv). Please pay attention to the last two arguments: "**coord**" is 0 for motion with respect to (w.r.t.) Arm base coordinate system, and 1 for motion w.r.t. Tool coordinate system. "**relative**" is 0 for absolute target position w.r.t. specified coordinate system, and 1 for relative target position.  
&ensp;&ensp;For example: to move 1.0 radian relatively around tool-frame Z-axis: 
```bash
$ rosservice call /xarm/move_line_aa "pose: [0, 0, 0, 0, 0, 1.0]
mvvelo: 30.0
mvacc: 100.0
mvtime: 0.0
coord: 1
relative: 1" 
ret: 0
message: "move_line_aa, ret = 0"
```
Or
```bash
$ rosservice call /xarm/move_line_aa [0,0,0,0,0,1.0] 30.0 100.0 0.0 1 1
```   
&ensp;&ensp;"**mvtime**" is not meaningful in this command, just set it to 0. Another example: in base-frame, to move 122mm relatively along Y-axis, and rotate around X-axis for -0.5 radians:  
```bash
$ rosservice call /xarm/move_line_aa [0,122,0,-0.5,0,0] 30.0 100.0 0.0 0 1  
```

##### 5. Joint velocity control:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling joint velocity is desired, first switch to **Mode 4** as descriped in [mode change section](#6-mode-change). Please check the [MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv) first to understand the meanings of parameters reqired. If more than one joint are to move, set **jnt_sync** to 1 for synchronized acceleration/deceleration for all joints in motion, and if jnt_sync is 0, each joint will reach to its target velocity as fast as possible. ***coord*** parameter is not used here, just set it to 0. For example: 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):
$ rosservice call /xarm/velo_move_joint [0.1,-0.1,0,0,0,-0.3] 1 0
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.2 seconds, xArm will stop)  
$ rosservice call /xarm/velo_move_joint_timed [0.1,-0.1,0,0,0,-0.3] 1 0 0.2
``` 
will command the joints (for xArm6) to move in specified angular velocities (in rad/s) and they will reach to target velocities synchronously. The maximum joint acceleration can also be configured by (unit: rad/s^2):  
```bash
$ rosservice call /xarm/set_max_acc_joint 10.0  (maximum: 20.0 rad/s^2)
``` 

##### 6. Cartesian velocity control:
&ensp;&ensp;(**xArm controller firmware version >= 1.6.8** required) If controlling linar velocity of TCP towards certain direction is desired, first switch to **Mode 5** as descriped in [mode change section](#6-mode-change). Please check the [MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv) first to understand the meanings of parameters reqired. Set **coord** to 0 for motion in world/base coordinate system and 1 for tool coordinate system. ***jnt_sync*** parameter is not used here, just set it to 0. For example: 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):  
$ rosservice call /xarm/velo_move_line [30,0,0,0,0,0] 0 1  
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.2 seconds, xArm will stop)  
$ rosservice call /xarm/velo_move_line_timed [30,0,0,0,0,0] 0 1 0.2
``` 
will command xArm TCP move along X-axis of TOOL coordinate system with speed of 30 mm/s. The maximum linear acceleration can also be configured by (unit: mm/s^2):  
```bash
$ rosservice call /xarm/set_max_acc_line 5000.0  (maximum: 50000 mm/s^2)
``` 

For angular motion in orientation, please note the velocity is specified as **axis-angular_velocity** elements. That is, [the unit rotation axis vector] multiplied by [rotation velocity value(scalar)]. For example, 
```bash
# NO Timed-out version (will not stop until all-zero velocity command received!):
$ rosservice call /xarm/velo_move_line [0,0,0,0.707,0,0] 0 0
# With Timed-out version(controller firmware version >= 1.8.0): (if next velocity command not received within 0.2 seconds, xArm will stop)  
$ rosservice call /xarm/velo_move_line_timed [0,0,0,0.707,0,0] 0 0 0.2
``` 
This will command TCP to rotate along X-axis in BASE coordinates at about 45 degrees/sec. The maximum acceleration for orientation change is fixed.  

Please Note: For no Timed-out version services: velocity motion can be stopped by either giving **all 0 velocity** command, or setting **state to 4(STOP)** and 0(READY) later for next motion. However, **timed-out versions are more recommended for use**, since it can be safe if network comminication or user program fails, controller firmware needs to be updated to v1.8.0 or later.  

#### Motion service Return:
&ensp;&ensp;Please Note the above motion services will **return immediately** by default. If you wish to return until actual motion is finished, set the ros parameter **"/xarm/wait_for_finish"** to be **true** in advance. That is:  
```bash
$ rosparam set /xarm/wait_for_finish true
```   
&ensp;&ensp;Upon success, 0 will be returned. If any error occurs, 1 will be returned.

#### Tool I/O Operations:

&ensp;&ensp;We provide 2 digital, 2 analog input port and 2 digital output signals at the end I/O connector.  
##### 1. To get current 2 DIGITAL input states:  
```bash
$ rosservice call /xarm/get_digital_in
```
##### 2. To get one of the ANALOG input value: 
```bash
$ rosservice call /xarm/get_analog_in 1  (last argument: port number, can only be 1 or 2)
```
##### 3. To set one of the Digital output:
```bash
$ rosservice call /xarm/set_digital_out 2 1  (Setting output 2 to be 1)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

#### Controller I/O Operations:

&ensp;&ensp;We provide 8/16 digital input and 8/16 digital output ports at controller box for general usage.  

##### 1. To get one of the controller DIGITAL input state:  
```bash
$ rosservice call /xarm/get_controller_din io_num (Notice: from 1 to 8, for CI0~CI7; from 9 to 16, for DI0~DI7[if any])  
```
##### 2. To set one of the controller DIGITAL output:
```bash
$ rosservice call /xarm/set_controller_dout io_num (Notice: from 1 to 8, for CO0~CO7; from 9 to 16, for DI0~DI7[if any]) logic (0 or 1) 
```
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_controller_dout 5 1  (Setting output 5 [lable C04] to be 1)
```
##### 3. To get one of the controller ANALOG input:
```bash
$ rosservice call /xarm/get_controller_ain port_num  (Notice: from 1 to 2, for AI0~AI1)
```
##### 4. To set one of the controller ANALOG output:
```bash
$ rosservice call /xarm/set_controller_aout port_num (Notice: from 1 to 2, for AO0~AO1) analog_value
```
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_controller_aout 2 3.3  (Setting port AO1 to be 3.3)
```
&ensp;&ensp;You have to make sure the operation is successful by checking responding "ret" to be 0.

#### Getting status feedback:
&ensp;&ensp;Having connected with a real xArm robot by running 'xarm7_server.launch', user can subscribe to the topic ***"xarm/xarm_states"*** for feedback information about current robot states, including joint angles, TCP position, error/warning code, etc. Refer to [RobotMsg.msg](./xarm_msgs/msg/RobotMsg.msg) for content details.  
&ensp;&ensp;Another option is subscribing to ***"/joint_states"*** topic, which is reporting in [JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html), however, currently ***only "position" field is valid***; "velocity" is non-filtered numerical differentiation based on 2 adjacent position data, and "effort" feedback are current-based estimated values, not from direct torque sensor, so they are just for reference.
&ensp;&ensp;In consideration of performance, default update rate of above two topics are set at ***5Hz***. The report content and frequency have other options, refer to [report_type argument](#report_type-argument)  

#### Setting Tool Center Point Offset(only effective for xarm_api ROS service control):
&ensp;&ensp;The tool tip point offset values can be set by calling service "/xarm/set_tcp_offset". Refer to the figure below, please note this offset coordinate is expressed with respect to ***default tool frame*** (Frame B), which is located at flange center, with roll, pitch, yaw rotations of (PI, 0, 0) from base frame (Frame A).   
![xArmFrames](./doc/xArmFrames.png)  
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp;This is to set tool frame position offset (x = 0 mm, y = 0 mm, z = 20 mm), and orientation (RPY) offset of ( 0, 0, 0 ) radians with respect to initial tool frame (Frame B in picture). ***Note this offset might be overwritten by xArm Stdudio if it is not consistent with the default value set in studio!*** It is recommended to do the same TCP default offset configuration in xArm studio if you want to use it alongside with ros service control.  

#### Clearing Errors:
&ensp;&ensp;Sometimes controller may report error or warnings that would affect execution of further commands. The reasons may be power loss, position/speed limit violation, planning errors, etc. It needs additional intervention to clear. User can check error code in the message of topic ***"xarm/xarm_states"*** . 
```bash
$ rostopic echo /xarm/xarm_states
```
&ensp;&ensp;If it is non-zero, the corresponding reason can be found out in the user manual. After solving the problem, this error satus can be removed by calling service ***"/xarm/clear_err"*** with empty argument.
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;If using Moveit!, call "**/xarm/moveit_clear_err**" instead to avoid the need of setting mode 1 again manually. 
```bash
$ rosservice call /xarm/moveit_clear_err
```

&ensp;&ensp;After calling this service, please ***check the err status again*** in 'xarm/xarm_states', if it becomes 0, the clearing is successful. Otherwise, it means the error/exception is not properly solved. If clearing error is successful, remember to ***set robot state to 0*** to make it ready to move again!   

#### Gripper Control:
&ensp;&ensp; If xArm Gripper (from UFACTORY) is attached to the tool end, the following services/actions can be called to operate or check the gripper.  

##### 1. Gripper services:  
(1) First enable the griper and configure the grasp speed:  
```bash
$ rosservice call /xarm/gripper_config 1500
```
&ensp;&ensp; Proper range of the speed is ***from 1 to 5000***. 1500 is used as an example. 'ret' value is 0 for success.  
(2) Give position command (open distance) to xArm gripper:  
```bash
$ rosservice call /xarm/gripper_move 500
```
&ensp;&ensp; Proper range of the open distance is ***from 0 to 850***. 0 is closed, 850 is fully open. 500 is used as an example. 'ret' value is 0 for success.  

(3) To get the current status (position and error_code) of xArm gripper:
```bash
$ rosservice call /xarm/gripper_state
```
&ensp;&ensp; If error code is non-zero, please refer to user manual for the cause of error, the "/xarm/clear_err" service can still be used to clear the error code of xArm Gripper.  

##### 2. Gripper action:
&ensp;&ensp; The xArm gripper move action is defined in [Move.action](/xarm_gripper/action/Move.action). The goal consists of target pulse position and the pulse speed. By setting "true" of "**use_gripper_action**" argument in xarm_bringup/launch/xarm7_server.launch, the action server will be started. Gripper action can be called by:  
```bash
$ rostopic pub -1 /xarm/gripper_move/goal xarm_gripper/MoveActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  target_pulse: 500.0
  pulse_speed: 1500.0"

```
&ensp;&ensp; Alternatively:
```bash
$ rosrun xarm_gripper gripper_client 500 1500 
```

#### Vacuum Gripper Control:
&ensp;&ensp; If Vacuum Gripper (from UFACTORY) is attached to the tool end, the following service can be called to operate the vacuum gripper.  

&ensp;&ensp;To turn on:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 1
```
&ensp;&ensp;To turn off:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 0
```
&ensp;&ensp;0 will be returned upon successful execution.  


#### Tool Modbus communication:
If modbus communication with the tool device is needed, please first set the proper baud rate and timeout parameters through the "xarm/config_tool_modbus" service (refer to [ConfigToolModbus.srv](/xarm_msgs/srv/ConfigToolModbus.srv)). For example: 
```bash
$ rosservice call /xarm/config_tool_modbus 115200 20
```
The above command will configure the tool modbus baudrate to be 115200 bps and timeout threshold to be 20 **ms**. It is not necessary to configure again if these properties are not changed afterwards. **Please note** the first time to change the baud rate may return 1 (with error code 28), in fact it will succeed if the device is properly connected and there is no other exsisting controller errors. You can clear the error and call it once more to check if 0 is returned. Currently, only the following baud rates (bps) are supported: [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000].  

Then the communication can be conducted like (refer to [SetToolModbus.srv](/xarm_msgs/srv/SetToolModbus.srv)):  
```bash
$ rosservice call /xarm/set_tool_modbus [0x01,0x06,0x00,0x0A,0x00,0x03] 6
```
First argument would be the uint8(unsigned char) data array to be sent to the modbus tool device, and second is the number of characters to be received as a response from the device. **This number should be the expected data byte length (without CRC bytes)**. For example, with some testing device the above instruction would reply:  
```bash
ret: 0
respond_data: [1, 6, 0, 10, 0, 3]
```
and actual feedback data frame is: [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03], with the length of 6 bytes.   

#### "report_type" argument:
When launching real xArm ROS applications, the argument "report_type" can be specified. It decides the state feedback rate and content. Refer to the [developer manual](https://www.ufactory.cc/_files/ugd/896670_1f106918b523404284c6916de025cf28.pdf) at chapter **2.1.6 Automatic Reporting Format** for the report contents of the three available report type (`normal/rich/dev`), default type using is "normal".  

* For users who demand high-frequency feedback, `report_type:=dev` can be specified, then the topics `/xarm/xarm_states` and `/xarm/joint_states` will be published at **100Hz**.  
* For users who want the gpio states being updated at `/xarm/controller_gpio_states` topic, please use `report_type:=rich`, since this reports the fullest information from the controller. As can be seen in developer manual.  
* The report rate of the three types: 

|   type   |    port No.   | Frequency |  GPIO topic   | F/T sensor topic | 
|:--------:|:-------------:|:---------:|:-------------:|:----------------:|
|   normal |     30001     |    5Hz    | Not Available |   Not Available  |
|   rich   |     30002     |    5Hz    |   Available   |     Available    | 
|   dev    |     30003     |    100Hz  | Not Available |     Available    |

Note: **GPIO topic** => `xarm/controller_gpio_states`. **F/T sensor topic** =>  `xarm/uf_ftsensor_ext_states` and `xarm/uf_ftsensor_raw_states`.

## 5.8 xarm_moveit_servo:
&ensp;&ensp;This package serves as a demo for jogging xArm with devices such as joystick.
   - #### 5.8.1 Controlling with XBOX360 joystick
      - left stick for X and Y direction.  
      - right stick for ROLL and PITCH adjustment.  
      - left and right trigger (LT/RT) for Z direction.  
      - left and right bumper (LB/RB) for YAW adjustment.  
      - D-PAD for controlling joint1 and joint2.  
      - buttons X and B for controlling last joint.  
      - buttons Y and A for controlling second last joint. 

      ```bash
      # For controlling real xArm: (use xArm 6 as example)
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=1
      # XBOX Wired -> joystick_type=1
      # XBOX Wireless -> joystick_type=2

      # Or controlling real Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=1 robot_type:=lite
      ```

   - #### 5.8.2 Controlling with 3Dconnexion SpaceMouse Wireless
      - 6 DOFs of the mouse are mapped for controlling X/Y/Z/ROLL/PITCH/YAW
      - Left button clicked for just X/Y/Z adjustment
      - Right button clicked for just ROLL/PITCH/YAW adjustment

      ```bash
      # For controlling real xArm: (use xArm 6 as example)
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=3

      # Or controlling real Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=3 robot_type:=lite
      ```

   - #### 5.8.3 Controlling with PC keyboard
      ```bash
      # For controlling real xArm: (use xArm 6 as example)
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=99

      # Or controlling real Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=99 robot_type:=lite
      ```

# 6. Mode Change
&ensp;&ensp;xArm may operate under different modes depending on different controling methods. Current mode can be checked in the message of topic "xarm/xarm_states". And there are circumstances that demand user to switch between operation modes. 

### 6.1 Mode Explanation

&ensp;&ensp; ***Mode 0*** : xArm controller (Position) mode.  
&ensp;&ensp; ***Mode 1*** : External trajectory planner (position) mode.  
&ensp;&ensp; ***Mode 2*** : Free-Drive (zero gravity) mode.  
&ensp;&ensp; ***Mode 3*** : Reserved.  
&ensp;&ensp; ***Mode 4*** : Joint velocity control mode.  
&ensp;&ensp; ***Mode 5*** : Cartesian velocity control mode.  
&ensp;&ensp; ***Mode 6*** : Joint space online planning mode. (Firmware >= v1.10.0)  
&ensp;&ensp; ***Mode 7*** : Cartesian space online planning mode. (Firmware >= v1.11.0)  

&ensp;&ensp;***Mode 0*** is the default when system initiates, and when error occurs(collision, overload, overspeed, etc), system will automatically switch to Mode 0. Also, all the motion plan services in [xarm_api](./xarm_api/) package or the [SDK](https://github.com/xArm-Developer/xArm-Python-SDK) motion functions demand xArm to operate in Mode 0. ***Mode 1*** is for external trajectory planner like Moveit! to bypass the integrated xArm planner to control the robot. ***Mode 2*** is to enable free-drive operation, robot will enter Gravity compensated mode, however, be sure the mounting direction and payload are properly configured before setting to mode 2. ***Mode 4*** is to control arm velocity in joint space. ***Mode 5*** is to control arm (linear) velocity in Cartesian space. ***Mode 6 and 7*** are for dynamic realtime response to newly generated joint or Cartesian target respectively, with automatic speed-continuous trajectoty re-planning.

### 6.2 Proper way to change modes:  
&ensp;&ensp;If collision or other error happens during the execution of a Moveit! planned trajectory, Mode will automatically switch from 1 to default mode 0 for safety purpose, and robot state will change to 4 (error state). The robot will not be able to execute any Moveit command again unless the mode is set back to 1. The following are the steps to switch back and enable Moveit control again:  

&ensp;&ensp;(1) Make sure the objects causing the collision are removed.  
&ensp;&ensp;(2) clear the error:  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) switch to the desired mode (Mode 2 for example), and set state to 0 for ready:
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```

# 7. xArm Vision
For simple demonstrations of vision application development with xArm, including hand-eye calibration and object detection and grasping. Examples are based on [Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/) depth camera.

## 7.1 Installation of dependent packages:

First enter the workspace source directory:
```bash
$ cd ~/catkin_ws/src/
```

### 7.1.1 Install RealSense developer library and ROS package： 
Please refer to the installation steps at [official webpage](https://github.com/IntelRealSense/realsense-ros).

### 7.1.2 Install 'aruco_ros', for hand-eye calibration：
Refer to [official Github](https://github.com/pal-robotics/aruco_ros):
```bash
$ git clone -b kinetic-devel https://github.com/pal-robotics/aruco_ros.git
```
### 7.1.3 Install 'easy_handeye', for hand-eye calibration：
Refer to [official Github](https://github.com/IFL-CAMP/easy_handeye):
```bash
$ git clone https://github.com/IFL-CAMP/easy_handeye
``` 
### 7.1.4 Install 'find_object_2d', for object detection：
Refer to [official Github](https://github.com/introlab/find-object/tree/kinetic-devel):
```bash
$ sudo apt-get install ros-kinetic-find-object-2d
```
### 7.1.5 Install other dependencies：
```bash
$ cd ~/catkin_ws
```
Then follow chapter [4.3](#43-install-other-dependent-packages).

### 7.1.6 Build the whole workspace：
```bash
$ catkin_make
```

## 7.2 Hand-eye Calibration Demo：
If attaching RealSense D435i camera at tool end of xArm, with mechanical adapters, making a "**eye-on-hand**"(or eye-in-hand) configuration，the following launch file can be used and modified for hand-eye calibration: (make sure the camera communication is functional and robot is properly switched on)
```bash
$ roslaunch d435i_xarm_setup d435i_xarm_auto_calib.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
Note: for xArm/UF850 produced **after August 2023**, kinematic calibration can be added to the URDF model, you can specify `kinematics_suffix` parameter for better accuracy, refer [here](https://github.com/xArm-Developer/xarm_ros/blob/master/uf_robot_moveit_config/Readme.md#optional-parameters) for details.   

The `aruco Marker` used inside can be downloaded [here](https://chev.me/arucogen/), please remember the `marker ID` and `marker size` and modify them in the launch file accordingly. Refer to [official](https://github.com/IFL-CAMP/easy_handeye#calibration)or other usage instructions online and finish the calibration with the GUI.   

If calculation result is confirmed and saved，it will appear by default under `~/.ros/easy_handeye` directory and can be used for transferring object coordinates to base frame. If the [camera_stand](https://www.ufactory.cc/products/xarm-camera-module-2020) provided by UFACTORY is used for fixing camera, a sample calibration result is stored at xarm_vision/d435i_xarm_setup/config/[xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml](./xarm_vision/d435i_xarm_setup/config/xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml) for this case.  


### 7.2.1 Hand-eye Calibration Demo for UFACTORY Lite6:
Please first read through the above instruction for xarm series, and use the files listed here for Lite6 calibration.  

Sample calibration launch:  
```bash
$ roslaunch d435i_xarm_setup d435i_lite6_auto_calib.launch robot_ip:=your_xArm_IP
```
Sample Calibration result file: [lite6_realsense_handeyecalibration_eye_on_hand_sample_result.yaml](./xarm_vision/d435i_xarm_setup/config/lite6_realsense_handeyecalibration_eye_on_hand_sample_result.yaml)

Sample calibration result TF publication:
[publish_handeye_tf_lite6.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf_lite6.launch)


### 7.2.2 Precautions for Hand-eye Calibration:
Since the position and orientation of the robot arm generated by `easy_handeye` by default does not change much, the final calibration result may not be too accurate or stable. 
In the actual calibration process, we do not need to use the position generated by `easy_handeye`. We can specify the startup parameter `freehand_robot_movement:=true` in the above command to start, and control the robotic arm to different positions through the xarm studio control interface or enable drag teaching. Then collect data through "__Take Sample__" of the activated hand-eye calibration window. After collecting about 17 data, calculate it through "__Compute__". It is recommended to rotate the rpy as much as possible to ensure that the calibration plate is within the field of view for the position of the robotic arm during each acquisition.
- The angle between the rotation axes of the two movements should be as large as possible
- The rotation angle corresponding to the rotation matrix of each movement should be as large as possible
- The distance from the camera center to the calibration plate should be as small as possible
- The distance moved by the end of the robot arm in each movement should be as small as possible

## 7.3 Vision Guided Grasping Demo:
[***find_object_2d***](http://introlab.github.io/find-object/) is used for this demo for simple object detection and grasping. Hardware used in this part: RealSense D435i depth camera, UFACTORY camera stand and the xArm Gripper.  

1.Use moveit to drive xArm's motion，recommended for singularity and collision free execution, but will require a reliable network connection.  
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_moveit_planner.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
If target object can be properly detected, to run the Grasping node:  
```bash
$ rosrun d435i_xarm_setup findobj2d_grasp_moveit
```

Please note it will use previously mentioned sample handeye calibration result, you can change it at [publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch). For node program source code, refer to: d435i_xarm_setup/src/[findobj_grasp_moveit_planner.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_planner.cpp).  

2.Alternatively, to drive xArm motion with ros service provided by 'xarm_api', in this way, real-time performance of network will not be required so strict as moveit way, but execution may fail in the middle if singularity or self-collision is about to occur. 
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_api.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
If target object can be properly detected, to run the Grasping node:  
```bash
$ roslaunch d435i_xarm_setup grasp_node_xarm_api.launch
```
Please note it will use previously mentioned sample handeye calibration result, you can change it at [publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch). For node program source code, refer to: d435i_xarm_setup/src/[findobj_grasp_xarm_api.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_xarm_api.cpp).  

***Please read and comprehend the source code and make necessary modifications before real application test***, necessary modifications include preparation pose, grasping orientation, grasping depth, motion speed and so on. The identification target name in the code is "object_1", which corresponds to `1.png` in /objects directory, users can add their own target in "find_object_2d" GUI, then modify the `source_frame` inside the code, for costomized application.  

***Tips***: make sure the background is clean and the color is distinguished from the object, detection success rate can be higher if the target object has rich texture (features).

## 7.4 Adding RealSense D435i model to simulated xArm：
For installation with camera stand provided by UFACTORY, the cam model can be attached by following modifications (use xarm7 as example):    
```bash
 $ roslaunch xarm7_moveit_config demo.launch add_realsense_d435i:=true
```

## 7.5 Color Cube Grasping Demo

### 7.5.1 Download 'gazebo_grasp_plugin' for successful grasp simulation (ROS Melodic and later)
```bash
 # enter source directory of ROS workspace:
 $ cd ~/catkin_ws/src/
 # Download through git (mind to checkout the proper branch):
 $ git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
 $ git clone https://github.com/JenniferBuehler/general-message-pkgs.git
 # Compile:
 $ cd ..
 $ catkin_make
```
### 7.5.2 Gazebo grasping simulation (ROS Melodic and later)
```bash
 # Initialize gazebo scene and move_group:
 $ roslaunch xarm_gazebo xarm_camera_scene.launch robot_dof:=6

 # In another terminal, run the color recognition and grasping script:
 $ rosrun xarm_gazebo color_recognition.py
```
### 7.5.3 Real xArm and Intel realsense_d435i hardware
```bash
 # launch move_group:
 $ roslaunch camera_demo xarm_move_group.launch robot_ip:=192.168.1.15 robot_dof:=6 add_realsense_d435i:=true

 # In another terminal, run the color recognition and grasping script (use with interaction prompt):
 $ rosrun camera_demo color_recognition.py
```


# 8. Other Examples
&ensp;&ensp;There are some other application demo examples in the [example package](./examples), which will be updated in the future, feel free to explore it.