## Updates on Moveit with Gripper/Vacuum Gripper:
&ensp;&ensp;Please pay attention if you are using Moveit motion planning for xArm models with Gripper/Vacuum Gripper attached, in updates since **Jan 6, 2021**, pose command/feedback will count in the **TCP offset** of the tool, as a result in moveit configuration change. 

## Important Notice:
&ensp;&ensp;Due to robot communication data format change, ***early users*** (xArm shipped ***before June 2019***) are encouraged to ***upgrade*** their controller firmware immediately to drive the robot normally in future updates as well as to use newly developed functions. Please contact our staff to get instructions of the upgrade process. The old version robot driver can still be available in ***'legacy'*** branch, however, it will not be updated any more.   

&ensp;&ensp;You MUST follow **chapter 3** to install additional packages needed before any usage of xarm_ros packages. Otherwise, unexpected errors may occur.

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
    * [5.6 ***xarm_planner(Updated)***](#56-xarm_planner)  
    * [5.7 ***xarm_api/xarm_msgs***](#57-xarm_apixarm_msgs)  
        * [5.7.1 Starting xArm by ROS service (***priority for the following operations***)](#starting-xarm-by-ros-service)  
        * [5.7.2 Joint space or Cartesian space command example(**Updated**)](#joint-space-or-cartesian-space-command-example)
        * [5.7.3 Tool/Controller I/O Operations](#tool-io-operations)  
        * [5.7.4 Getting status feedback](#getting-status-feedback)  
        * [5.7.5 Setting Tool Center Point Offset](#setting-tool-center-point-offset)  
        * [5.7.6 Clearing Errors](#clearing-errors)  
        * [5.7.7 Gripper Control](#gripper-control)
        * [5.7.8 Vacuum Gripper Control](#vacuum-gripper-control)
        * [5.7.9 Tool Modbus communication](#tool-modbus-communication)
* [6. Mode Change](#6-mode-change)
    * [6.1 Mode Explanation](#61-mode-explanation)
    * [6.2 Proper way to change modes](#62-proper-way-to-change-modes)
* [7. Other Examples  (***Updated***)](#7-other-examples)
    * [7.1 Multi-xArm5 (separate control)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#1-multi_xarm5-controlled-separately)
    * [7.2 Servo_Cartesian](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)
    * [7.3 Servo_Joint](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory)
    * [7.4 Dual xArm6 controlled with one moveGroup node](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#4-dual-xarm6-controlled-with-one-movegroup-node)
    * [7.5 An example of demonstrating redundancy resolution using MoveIt](https://github.com/xArm-Developer/xarm_ros/tree/master/examples/xarm7_redundancy_res)

# 1. Introduction
   &ensp;&ensp;This repository contains the 3D models of xArm series and demo packages for ROS development and simulations.Developing and testing environment: Ubuntu 16.04 + ROS Kinetic Kame + Gazebo 9.  
   ***Instructions below is based on xArm7, other model user can replace 'xarm7' with 'xarm6' or 'xarm5' where applicable.***
   For simplified Chinese instructions: [简体中文版](./ReadMe_cn.md)    

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

12/22/2020: Refer to issue #53, Please Note this plugin has recently been **deprecated**, if you plan to use [new version](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), please change "libroboticsgroup_gazebo_mimic_joint_plugin.so" to "libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so" in file: xarm_ros/xarm_gripper/urdf/xarm_gripper.gazebo.xacro  

# 4. Getting started with 'xarm_ros'
   
## 4.1 Create a catkin workspace. 
   &ensp;&ensp;If you already have a workspace, skip and move on to next part.
   Follow the instructions in [this page](http://wiki.ros.org/catkin/Tutorials/create_a_workspace). 
   Please note that this readme instruction assumes the user continues to use '~/catkin_ws' as directory of the workspace.

## 4.2 Obtain the package
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git
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
   And chane 'kinetic' to the ROS distribution you use.  

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
&ensp;&ensp;Partially generated by moveit_setup_assistant, could use with Moveit Planner and Rviz visualization. If you have Moveit! installed, you can try the demo. 
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
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   Examine the terminal output and see if any error occured during the launch. If not, just play with the robot in Rviz and you can execute the sucessfully planned trajectory on real arm. But be sure it will not hit any surroundings before execution!  

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

## 5.6 xarm_planner:
&ensp;&ensp;This implemented simple planner interface is based on move_group from Moveit! and provide ros service for users to do planning & execution based on the requested target, user can find detailed instructions on how to use it inside [***xarm_planner package***](./xarm_planner/).  
#### To launch the xarm simple motion planner together with the real xArm:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<your controller box LAN IP address> robot_dof:=<7|6|5> add_(vacuum_)gripper:=<true|false>
```
Argument 'robot_dof' specifies the number of joints of your xArm (default is 7). Now xarm_planner supports model with gripper or vacuum_gripper attached. Please specify "**add_gripper**" or "**add_vacuum_gripper**" argument if needed.    

## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;These two packages provide user with the ros service wrapper of the functions in xArm SDK. There are 6 types of motion command (service names) supported:  
* <font color=blue>move_joint:</font> joint space point to point command, given target joint angles, max joint velocity and acceleration. Corresponding function in SDK is "set_servo_angle()".  
* <font color=blue>move_line:</font> straight-line motion to the specified Cartesian Tool Centre Point(TCP) target. Corresponding function in SDK is "set_position()"[blending radius not specified].  
* <font color=blue>move_lineb:</font> a list of via points followed by target Cartesian point. Each segment is straight-line with Arc blending at the via points, to make velocity continuous. Corresponding function in SDK is "set_position()"[blending radius specified]. Please refer to [move_test.cpp](./xarm_api/test/move_test.cpp) for example code.   
* <font color=blue>move_line_tool:</font> straight-line motion based on the **Tool coordinate system** rather than the base system. Corresponding function in SDK is "set_tool_position()".  
Please ***keep in mind that*** before calling the 4 motion services above, first set robot mode to be 0, then set robot state to be 0, by calling relavent services. Meaning of the commands are consistent with the descriptions in product ***user manual***, other xarm API supported functions are also available as service call. Refer to [xarm_msgs package](./xarm_msgs/) for more details and usage guidance.  

* <font color=blue>move_line_aa:</font> straight-line motion, with orientation expressed in **Axis-angle** rather than roll-pitch-yaw angles. Please refer to xArm user manual for detailed explanation of axis-angle before using this command.   

* <font color=blue>move_servo_cart/move_servoj:</font> streamed high-frequency trajectory command execution in Cartesian space or joint space. Corresponding functions in SDK are set_servo_cartesian() and set_servo_angle_j(). An alternative way to implement <font color=red>velocity control</font>. These two services operate the robot in mode 1. Special **RISK ASSESMENT** is required before using them. Please read the guidance carefully at [chapter 7.2-7.3](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory).  


#### Starting xArm by ROS service:

&ensp;&ensp;First startup the service server for xarm7, ip address is just an example:  
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128
```
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

&ensp;&ensp;We provide 8 digital input and 8 digital output ports at controller box for general usage.  

##### 1. To get one of the controller DIGITAL input state:  
```bash
$ rosservice call /xarm/get_controller_din io_num (Notice: from 1 to 8, for CI0~CI7)  
```
##### 2. To set one of the controller DIGITAL output:
```bash
$ rosservice call /xarm/set_controller_dout io_num (Notice: from 1 to 8, for CO0~CO7) logic (0 or 1) 
```
&ensp;&ensp;For example:  
```bash
$ rosservice call /xarm/set_controller_dout 5 1  (Setting output 5 to be 1)
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
&ensp;&ensp;In consideration of performance, current update rate of above two topics are set at ***5Hz***.  

#### Setting Tool Center Point Offset:
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
$ rosservice call /xarm/set_tool_modbus [0x01,0x06,0x00,0x0A,0x00,0x03] 7
```
First argument would be the uint8(unsigned char) data array to be sent to the modbus tool device, and second is the number of characters to be received as a response from the device. **This number should be the expected data byte length +1 (without CRC bytes)**. A byte with value of **0x09** would always be attached to the head if received from tool modbus, and the rest bytes are response data from the device. For example, with some testing device the above instruction would reply:  
```bash
ret: 0
respond_data: [9, 1, 6, 0, 10, 0, 3]
```
and actual feedback data frame is: [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03], with the length of 6 bytes.   


# 6. Mode Change
&ensp;&ensp;xArm may operate under different modes depending on different controling methods. Current mode can be checked in the message of topic "xarm/xarm_states". And there are circumstances that demand user to switch between operation modes. 

### 6.1 Mode Explanation

&ensp;&ensp; ***Mode 0*** : xArm controller (Position) Mode.  
&ensp;&ensp; ***Mode 1*** : External trajectory planner (position) Mode.  
&ensp;&ensp; ***Mode 2*** : Free-Drive (zero gravity) Mode.  

&ensp;&ensp;***Mode 0*** is the default when system initiates, and when error occurs(collision, overload, overspeed, etc), system will automatically switch to Mode 0. Also, all the motion plan services in [xarm_api](./xarm_api/) package or the [SDK](https://github.com/xArm-Developer/xArm-Python-SDK) motion functions demand xArm to operate in Mode 0. ***Mode 1*** is for external trajectory planner like Moveit! to bypass the integrated xArm planner to control the robot. ***Mode 2*** is to enable free-drive operation, robot will enter Gravity compensated mode, however, be sure the mounting direction and payload are properly configured before setting to mode 2.

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
&ensp;&ensp;The above operations can also be done by calling relavant xArm SDK functions.   

***Known issue:*** Due to current controller (v1.6.5 and before) logic, ***it will fail hen switching directly between mode 1 and mode 2, you have to switch to mode 0 first then switch again to another non-zero mode.*** We will try to fix this issue in next controller firmware updates.  


# 7. Other Examples
&ensp;&ensp;There are some other application demo examples in the [example package](./examples), which will be updated in the future, feel free to explore it.
