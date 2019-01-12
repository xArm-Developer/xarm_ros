# 1. 简介：
   &ensp;&ensp;此代码库包含XArm模型文件以及相关的控制、规划等示例开发包。开发及测试使用的环境为 Ubuntu 16.04 + ROS Kinetic Kame。
   维护者: Jason (jason@ufactory.cc),Jimy (jimy.zhang@ufactory.cc)   
   ***以下的指令说明是基于xArm7, 其他型号用户可以在对应位置将'xarm7'替换成'xarm6'或'xarm5'***

# 2. 更新记录：
   此代码库仍然处在早期开发阶段，新的功能支持、示例代码，bug修复等等会保持更新。  
   * 添加Xarm 7 描述文档，3D图形文件以及controller示例，用于进行ROS可视化仿真模拟。
   * 添加MoveIt!规划器支持，用于控制Gazebo/RViz模型或者XArm真机，但二者不可同时启动。
   * 由ROS直接控制XArm真机的相关支持目前还是Beta版本，用户使用时应尽量小心，我们会尽快完善。
   * 添加 xArm hardware interface 并在驱动真实机械臂时使用 ROS position_controllers/JointTrajectoryController。
   * 添加 xArm 6 初版仿真支持。

# 3. 准备工作

## 3.1 安装 gazebo_ros interface 模块
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing>  
   ros_control: <http://wiki.ros.org/ros_control> (记得选择您使用的 ROS 版本)  
   
## 3.2 完整学习相关的官方教程
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 3.3 如果使用Gazebo: 请提前下载好 'table' 3D 模型
&ensp;&ensp;这个模型在Gazebo demo中会用到。在Gazebo仿真环境中, 在model database列表里寻找 'table', 并将此模型拖入旁边的3D环境中. 通过这个操作，桌子的模型就会自动下载到本地。

# 4. 'xarm_ros'的使用教程
   
## 4.1 生成catkin workspace. 
   &ensp;&ensp;如果您已经有了自己的catkin工作区，请跳过此步往下进行。
   按照[这里](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)的教程生成catkin_ws。 
   请留意本文档已假设用户继续沿用 '~/catkin_ws' 作为默认的catkin工作区地址。

## 4.2 获取代码包
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git
   ```

## 4.3 编译代码
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.4 执行配置脚本
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
如果在您的 ~/.bashrc中已经有了以上语句，直接运行:
```bash
$ source ~/.bashrc
```
## 4.5 在RViz环境试用:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```
## 4.6 如果已安装Gazebo,可以执行demo查看效果
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true]
   ```
&ensp;&ensp;指定'run_demo'为true时Gazebo环境启动后机械臂会自动执行一套编好的循环动作。 这套简单的command trajectory写在xarm_controller\src\sample_motion.cpp. 这个demo加载的控制器使用position interface（纯位置控制）。

# 5. 代码库介绍及使用说明
   
## 5.1 xarm_description
   &ensp;&ensp;包含xArm描述文档, mesh文件和gazebo plugin配置等等。 不推荐用户去修改urdf描述因为其他ros package对其都有依赖。

## 5.2 xarm_gazebo
   &ensp;&ensp;Gazebo world 描述文档以及仿真launch启动文档。用户可以在world中修改添加自己需要的模型与环境。

## 5.3 xarm_controller
   &ensp;&ensp;xarm使用的Controller配置, 硬件接口，轨迹指令源文件, 脚本以及launch文件。 用户可以基于这个包开发或者使用自己的package。***注意*** xarm_controller/config里面定义好的position/effort控制器仅用作仿真的示例, 当控制真实机械臂时只提供position_controllers/JointTrajectoryController接口。用户可以根据需要添加自己的controller, 参考: http://wiki.ros.org/ros_control (controllers)

## 5.4 xarm_bringup  
&ensp;&ensp;内含加载xarm driver的启动文件，用来控制真实机械臂。  

## 5.5 xarm7_moveit_config
&ensp;&ensp;
   部分文档由moveit_setup_assistant自动生成, 用于Moveit Planner和Rviz可视化仿真。如果已安装MoveIt!,可以尝试跑demo： 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```

#### Moveit!图形控制界面 + Gazebo 仿真环境:  
   首先执行:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   然后在另一个终端运行:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   如果您在Moveit界面中规划了一条满意的轨迹, 点按"Execute"会使Gazebo中的虚拟机械臂同步执行此轨迹。

#### Moveit!图形控制界面 + xArm 真实机械臂:
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<控制盒的局域网IP地址>
   ```
   检查terminal中的输出看看有无错误信息。如果启动无误，您可以将RViz中通过Moveit规划好的轨迹通过'Execute'按钮下发给机械臂执行。***但一定确保它不会与周围环境发生碰撞！***
  

## 5.6 xarm_planner:
这个简单包装实现的规划器接口是基于 Moveit!中的 move_group interface, 可以使用户通过service指定目标位置进行规划和执行。 这部分的详细使用方法请阅读[xarm_planner包](./xarm_planner)的文档。  
#### 启动 xarm simple motion planner 控制 xArm 真实机械臂:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<控制盒的局域网IP地址> robot_dof:=<7/6/5>
```
'robot_dof'参数指的是xArm的关节数目 (默认值为7)。  

## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;这两个package提供给用户不需要自己进行轨迹规划(通过Moveit!或xarm_planner)就可以控制真实xArm机械臂的ros服务, xarm自带的控制盒会进行轨迹规划。 请 ***注意*** 这些service的执行并不通过面向'JointTrajectoryController'的hardware interface。当前支持三种运动命令（ros service同名）:  
* move_joint: 关节空间的点到点运动, 用户仅需要给定目标关节位置，运动过程最大关节速度/加速度即可。 
* move_line: 笛卡尔空间的直线轨迹运动，用户需要给定工具中心点（TCP）目标位置以及笛卡尔速度、加速度。  
* move_lineb: 圆弧交融的直线运动，给定一系列中间点以及目标位置。 每两个中间点间为直线轨迹，但在中间点处做一个圆弧过渡（需给定半径）来保证速度连续。
另外需要 ***注意*** 的是，使用以上三种service之前，需要通过service依次将机械臂模式(mode)设置为0，然后状态(state)设置为0。这些运动指令的意义和详情可以参考产品使用指南。除此之外还提供了其他xarm编程API支持的service调用, 对于相关ros service的定义在 [xarm_msgs目录](./xarm_msgs/)中。 

#### 使用 API service call 的示例:

&ensp;&ensp;首先启动xarm7 service server, 以下ip地址只是举例:  
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128
```
&ensp;&ensp;然后确保每个关节的控制已经使能, 参考[SetAxis.srv](/xarm_msgs/srv/SetAxis.srv):
```bash
$ rosservice call /xarm/motion_ctrl 8 1
```
&ensp;&ensp;在传递任何运动指令前，先***依次***设置正确的机械臂模式(0: POSE)和状态(0: READY), 参考[SetInt16.srv](/xarm_msgs/srv/SetInt16.srv):    
```bash
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```
&ensp;&ensp;以上三个运动命令使用同类型的srv request: [Move.srv](./xarm_msgs/srv/Move.srv)。 比如，调用关节运动命令，最大速度 0.35 rad/s，加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;调用笛卡尔空间指令，最大线速度 200 mm/s，加速度为 2000 mm/s^2:
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
&ensp;&ensp;调用回原点服务 (各关节回到0角度)，最大角速度 0.35 rad/s，角加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```
#### 获得反馈状态信息:
&ensp;&ensp;如果通过运行'xarm7_server.launch'连接了一台xArm机械臂，用户可以通过订阅 ***"/xarm_status"*** topic 获得机械臂当前的各种状态信息， 包括关节角度、工具坐标点的位置、错误、警告信息等等。具体的信息列表请参考[RobotMsg.msg](./xarm_msgs/msg/RobotMsg.msg).  
&ensp;&ensp;另一种选择是订阅 ***"/joint_states"*** topic, 它是以[JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html)格式发布数据的, 但是当前 ***只有 "position" 是有效数据***; "velocity" 是没有经过任何滤波的基于相邻两组位置数据进行的数值微分, 因而只能作为参考，我们目前还不提供 "effort" 的反馈数据.
&ensp;&ensp;基于运行时性能考虑，目前以上两个topic的数据更新率固定为 ***10Hz***.  

#### 关于设定末端工具偏移量:  
&ensp;&ensp;末端工具的偏移量可以也通过'/xarm/set_tcp_offset'服务来设定,参考下图，请注意这一坐标偏移量是基于 ***原始工具坐标系*** (坐标系B)描述的，它位于末端法兰中心，并且相对基坐标系(坐标系A)有（PI, 0, 0)的RPY旋转偏移。
![xArmFrames](./doc/xArmFrames.png)  
&ensp;&ensp;例如：
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp;这条命令设置了基于原始工具坐标系(x = 0 mm, y = 0 mm, z = 20 mm)的位置偏移量，还有（0 rad, 0 rad, 0 rad)的RPY姿态偏移量。***如果需要请在每次重新启动/上电控制盒时设定一次正确的偏移量，因为此设定会掉电丢失。***