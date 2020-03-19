# 1. multi_xarm5
This package might give you a hint about how to launch and control two or more xArms (same model), while avoiding name clashes.Here xArm5 is used as an example, you can edit the launch files to control xarm6 or xarm7.     

(1) For a simulation, run:
```bash
$ roslaunch examples two_sim_xarm5.launch ns:=/uf
```
As an example, this command will bring up 2 xArm5 simulation models in 2 separate Moveit-RViz application windows. You can see in 'rqt_graph' that they come with 2 namespaces, one is '/uf' and the other is '/uf_2', and they can be controlled independently.  

(2) Or if two real xArm5 are available, try:
```bash
$ roslaunch examples two_real_xarm5.launch ns_1:=/left robot_ip_1:=192.168.1.212 ns_2:=/right robot_ip_2:=192.168.1.233
```
The IP addresses are just for examples here, if properly launched, you can control 2 real xarm5 in two separate Moveit-RViz windows independently.  

# 2. Servo_Cartesian (streamed Cartesian trajectory)
Now with xArm controller **Firmware Version >= 1.4.1**, user can send streamed Cartesian poses to continuously control the xArm, through ros service, this is a wrap of the same xarm sdk(api) function. The actual speed depends on the sending frequency and step distance. Users are encouraged to send the path points in a fixed frequency(20Hz~100Hz) and step distance(**MUST** <10mm).   
(1) To start, bring up the xarm server. [use xArm 7 as example]
```bash
$  roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.233
```
(2) In another terminal, do initial configurations, servo cartesian has to operate in **Mode 1**:
```bash
$ rosservice call /xarm/motion_ctrl 8 1
$ rosservice call /xarm/set_mode 1
$ rosservice call /xarm/set_state 0
```
(3) Call the servo_cartesian service. Please note:  
**(1) the Cartesian pose representation is the same with xarm SDK here. which is [X(mm), Y(mm), Z(mm), Roll(rad), Pitch(rad), Yaw(rad)]**  
**(2) the path must start from current tool center point(TCP) position and the command can not be too far away, or the execution will fail or act strange. PLEASE CHECK the correctness of command before sending it.**  
**(3) service argument format is like: "pose: [214, 0, 121, 3.1416, 0, 0]	mvvelo: 0.0	mvacc: 0.0	mvtime: 0.0	mvradii: 0.0"**    
**(4) the arguments mvvelo, mvacc, mvtime, and mvradii are not effective now, so just give them all 0.**  
  
Suppose current TCP position is at [206, 0, 121, 3.1416, 0, 0]
```bash
$ rosservice call /xarm/move_servo_cart [210,0,121,3.1416,0,0] 0.0 0.0 0.0 0.0
```
Now please check the current TCP position, it will execute this target immediately if success. If you want continuous motion alone X axis, you can give the following pose like:
```bash
$ rosservice call /xarm/move_servo_cart [214,0,121,3.1416,0,0] 0.0 0.0 0.0 0.0
```
And you can program this service calling procedure in a loop with proper intervals inbetween, the final execution will become smooth.   
