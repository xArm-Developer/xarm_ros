**UFACTORY Lite 6/850**用户, 确保您已经完成本篇说明中4.7节之前的部分，然后可以切换至[Lite6/UF850说明](./ReadMe_others.md).  
**kinetic**版本的用户，请使用[kinetic分支](https://github.com/xArm-Developer/xarm_ros/tree/kinetic).

## 重要提示:
&ensp;&ensp;使用xArm C++ SDK作为子模块后，**/xarm/set_tool_modbus**服务的使用有所修改，相比之前版本，回复中多余的0x09字节将**不再需要***！  
&ensp;&ensp;由于机械臂通信格式修改, 建议在***2019年6月前发货***的xArm 早期用户尽早 ***升级*** 控制器固件程序，这样才能在以后的更新中正常驱动机械臂运动以及使用最新开发的各种功能。请联系我们获得升级的详细指示。 当前ROS库主要的分支已不支持旧版本，先前版本的ROS驱动包还保留在 ***'legacy'*** 分支中, 但不会再有更新。    
&ensp;&ensp;在使用xarm_ros之前，请务必按照第3节**准备工作**的指示安装必要的第三方支持库，否则使用时会出现错误。  
&ensp;&ensp;如果使用**Moveit**开发, 请尽量在PC和控制器之间使用**网线直连方式**, 不要使用交换机等中间设备, 否则引入的通信延迟可能会对Moveit轨迹执行造成不良影响。  
&ensp;&ensp; 更新本仓库代码时, 请记得同时[检查submodule更新](#421-更新代码包)   

# 目录:  
* [1. 简介](#1-简介)
* [2. 更新记录](#2-更新记录)
* [3. 准备工作(***必须***)](#3-准备工作)
* [4. 开始使用'xarm_ros'](#4-开始使用xarm_ros)
* [5. 代码库介绍及使用说明](#5-代码库介绍及使用说明)
    * [5.1 xarm_description](#51-xarm_description)  
    * [5.2 xarm_gazebo](#52-xarm_gazebo)  
    * [5.3 xarm_controller](#53-xarm_controller)  
    * [5.4 xarm_bringup](#54-xarm_bringup)  
    * [5.5 ***xarm7_moveit_config***](#55-xarm7_moveit_config)  
        * [5.5.1 Moveit加载其它自定义模型到机械臂末端](#551-moveit加载其它自定义模型到机械臂末端)
    * [5.6 ***xarm_planner***](#56-xarm_planner)  
    * [5.7 ***xarm_api/xarm_msgs(新增关节/笛卡尔在线规划)***](#57-xarm_apixarm_msgs)  
        * [5.7.1 使用ROS Service启动 xArm (***后续指令执行的前提***)](#使用ros-service启动-xarm)  
        * [5.7.2 关节空间和笛卡尔空间运动指令的示例](#关节空间和笛卡尔空间运动指令的示例)
        * [5.7.3 I/O 操作](#工具-io-操作)  
        * [5.7.4 获得反馈状态信息](#获得反馈状态信息)  
        * [5.7.5 关于设定末端工具偏移量](#关于设定末端工具偏移量)  
        * [5.7.6 清除错误](#清除错误)  
        * [5.7.7 机械爪控制](#机械爪控制)  
        * [5.7.8 真空吸头控制](#真空吸头控制)  
        * [5.7.9 末端工具Modbus通信](#末端工具modbus通信)
        * [5.7.10 'report_type'启动参数](#report_type-启动参数)
    * [5.8 ***xarm_moveit_servo***](#58-xarm_moveit_servo)
* [6. 模式切换(***更新***)](#6-模式切换)
    * [6.1 模式介绍](#61-模式介绍)
    * [6.2 切换模式的正确方法](#62-切换模式的正确方法)
* [7. xArm视觉](#7-xArm视觉)
    * [7.1 相关依赖库和ros包的安装](#71-相关依赖库和ros包的安装)
    * [7.2 手眼标定示例](#72-手眼标定示例)
    * [7.3 3D视觉抓取示例](#73-3D视觉抓取示例)
    * [7.4 在仿真的xArm模型末端添加RealSense D435i模型](#74-在仿真的xarm模型末端添加realsense-d435i模型)
    * [7.5 颜色块抓取例子 (仿真+真机)](#75-颜色块抓取例子)
* [8. 其他示例](#8-其他示例)
    * [8.0 用Moveit展示xarm7冗余解的示例](https://github.com/xArm-Developer/xarm_ros/tree/master/examples/xarm7_redundancy_res)
    * [8.1 两台xArm5 (两进程独立控制)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#1-multi_xarm5-controlled-separately)
    * [8.2 Servo_Cartesian 笛卡尔位置伺服](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)
    * [8.3 Servo_Joint 关节位置伺服](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#3-servo_joint-streamed-joint-space-trajectory)
    * [8.4 使用同一个moveGroup节点控制xArm6双臂](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#4-dual-xarm6-controlled-with-one-movegroup-node)
    * [8.5 轨迹录制和回放](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#5-run-recorded-trajectory-beta)
    * [8.6 可用于动态跟踪的在线轨迹规划(**NEW**)](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)

# 1. 简介：
   &ensp;&ensp;此代码库包含xArm模型文件以及相关的控制、规划等示例开发包。开发及测试使用的环境为 Ubuntu 16.04 + ROS Kinetic/Melodic。
   ***以下的指令说明是基于xArm7, 其他型号用户可以在对应位置将'xarm7'替换成'xarm6'或'xarm5'***

# 2. 更新记录：
   此代码库仍然处在开发阶段，新的功能支持、示例代码，bug修复等等会保持更新。  
   * 添加xArm 7 描述文档，3D图形文件以及controller示例，用于进行ROS可视化仿真模拟。
   * 添加MoveIt!规划器支持，用于控制Gazebo/RViz模型或者xArm真机，但二者不可同时启动。
   * 添加 ROS直接控制xArm真机的相关支持，用户使用时应尽量小心。
   * 添加 xArm hardware interface 并在驱动真实机械臂时使用 ROS position_controllers/JointTrajectoryController。
   * 添加 xArm6 和 xArm5 仿真和真机控制支持。
   * 添加 xArm 机械爪仿真模型。
   * 添加 xArm6 双臂控制示例。
   * 添加 xArm Gripper action 控制。
   * 添加 xArm-with-gripper Moveit 开发包。
   * 添加 vacuum gripper（真空吸头）3D模型以及 xArm-with-vacuum-gripper Moveit开发包 (位于 /examples 路径下)。
   * 在[Microsoft IoT](https://github.com/ms-iot)的帮助下, xarm_ros 现已能够在 Windows平台编译和运行。
   * 添加关节空间和笛卡尔空间的速度控制模式。(需要**控制器固件版本 >= 1.6.8**)
   * 支持[Moveit添加其它模型到末端](#551-moveit加载其它自定义模型到机械臂末端)  
   * 添加带超时版本的速度模式控制。(需要**控制器固件版本 >= 1.8.0**)  
   * 添加xArm Vision和RealSense D435i相关demo，将之前的"xarm_device"内容转移至 xarm_vision/camera_demo。
   * xarm_controler (xarm_hw)不再通过service和topic的方式来使用SDK，而是直接调用SDK的接口
   * 支持Lite6模型
   * [Beta]新增连个力矩相关的主题（暂时不支持第三方力矩传感器）: /xarm/uf_ftsensor_raw_states(原始数据)和/xarm/uf_ftsensor_ext_states(经过滤波后的数据)
   * (2022-09-07) 新增service(__set_tgpio_modbus_timeout__/__getset_tgpio_modbus_data__)，根据参数选择是否透传Modbus数据
   * (2022-09-07) 更新子模块xarm-sdk到1.11.0版本
   * (2022-11-16) 增加力矩相关services: /xarm/ft_sensor_enable, /xarm/ft_sensor_app_set, /xarm/ft_sensor_set_zero, /xarm/ft_sensor_cali_load, /xarm/get_ft_sensor_error
   * (2023-02-10) 新增xarm_moveit_servo支持xbox手柄/SpaceMouse/键盘控制
   * (2023-02-18) 给service(/xarm/ft_sensor_cali_load)增加保存操作, 增加力矩相关service(/xarm/ft_sensor_iden_load)
   * (2023-02-27) 增加控制Lite6 Gripper的service(/ufactory/open_lite6_gripper, /ufactory/close_lite6_gripper, /ufactory/stop_lite6_gripper)(注： 一旦stop之后，close将无效，必须先open才能启用控制)
   * (2023-03-29) 新增launch启动参数model1300(默认为false), 更换xarm机械臂末端模型为1300系列的
   * (2023-04-20) 更新URDF文件，适配ROS1和ROS2，并根据SN从配置文件加载连杆的惯性参数
   * (2023-04-20) 新增launch启动参数`add_realsense_d435i`(默认为false), 支持加载Realsense D435i模型
   * (2023-04-20) 新增launch启动参数`add_d435i_links`(默认为false), 支持在加载RealSense D435i模型的情况下增加D435i的各摄像头之间的连杆关系，在`add_realsense_d435i`为true时才有用
   * (2023-04-20) 新增launch启动参数`robot_sn`支持加载对应的关节连杆的惯性参数，并自动覆盖model1300参数
   * (2023-04-20) 新增launch启动参数 `attach_to`/`attach_xyz`/`attach_rpy`，支持把机械臂模型依附在其它模型之上
   * (2023-04-21) 新增xarm_api里的[services使用说明文档](xarm_api/ReadMe.md)
   * (2023-06-07) 新增对UFACTORY850机械臂的支持
   * (2023-06-07) 新增 [uf_robot_moveit_config](uf_robot_moveit_config/Readme_cn.md), 支持xArm/Lite6/UFACTORY850系列机械臂使用moveit控制，后续可能会替代以下这些包，使用说明参见 [uf_robot_moveit_config](uf_robot_moveit_config/Readme_cn.md)
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
   * (2023-10-12) 新增关节kinematics参数文件的生成与使用 (仅支持 __uf_robot_moveit_config__，参见[uf_robot_moveit_config](uf_robot_moveit_config/Readme_cn.md)里的通用参数`kinematics_suffix`的说明)

# 3. 准备工作

## 3.1 安装依赖的模块
   gazebo_ros_pkgs: <http://gazebosim.org/tutorials?tut=ros_installing> （如果使用Gazebo模拟器）  
   ros_control: <http://wiki.ros.org/ros_control> (记得选择您使用的 ROS 版本)  
   moveit_core: <https://moveit.ros.org/install/>  
   
## 3.2 完整学习相关的官方教程
ROS Wiki: <http://wiki.ros.org/>  
Gazebo Tutorial: <http://gazebosim.org/tutorials>  
Gazebo ROS Control: <http://gazebosim.org/tutorials/?tut=ros_control>  
Moveit tutorial: <http://docs.ros.org/kinetic/api/moveit_tutorials/html/>  

## 3.3 如果使用Gazebo: 请提前下载好 'table' 3D 模型
&ensp;&ensp;这个模型在Gazebo demo中会用到。在Gazebo仿真环境中, 在model database列表里寻找 'table', 并将此模型拖入旁边的3D环境中. 通过这个操作，桌子的模型就会自动下载到本地。  

## 3.4 安装"mimic_joint_plugin"用于xArm Gripper的Gazebo仿真
&ensp;&ensp;如果xArm Gripper需要在Gazebo环境中仿真, 为了使物理引擎中的 mimic joints 并联机构可以正常运作，需要安装来自Konstantinos Chatzilygeroudis (@costashatz) 的 [**mimic_joint_plugin**](https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins)。关于这个插件的使用参考了@mintar的[这个教程](https://github.com/mintar/mimic_joint_gazebo_tutorial) 。  

12/22/2020: 参考issue #53, 请留意最近这个插件已经弃用不再支持, 如果您想要使用[新版本](https://github.com/roboticsgroup/roboticsgroup_upatras_gazebo_plugins), 请将xarm_ros/xarm_gripper/urdf/xarm_gripper.gazebo.xacro 文件中的"libroboticsgroup_gazebo_mimic_joint_plugin.so"替换为"libroboticsgroup_upatras_gazebo_mimic_joint_plugin.so"    

# 4. 开始使用'xarm_ros'
   
## 4.1 生成catkin workspace. 
   &ensp;&ensp;如果您已经有了自己的catkin工作区，请跳过此步往下进行。
   按照[这里](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)的教程生成catkin_ws。 
   请留意本文档已假设用户继续沿用 '~/catkin_ws' 作为默认的catkin工作区地址。

## 4.2 获取代码包
   ```bash
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/xArm-Developer/xarm_ros.git --recursive
   ```

## 4.2.1 更新代码包
   ```bash
   $ cd ~/catkin_ws/src/xarm_ros
   
   # 如果克隆时没有使用--recursive或--recurse-submodules参数, 使用该命令来初始化并更新所有子模块
   $ git submodule update --init --recursive
    
   # 拉取（Pull）主仓库并更新子模块
   $ git pull --recurse-submodules
   ```

## 4.3 安装其他依赖包:
   ```bash
   $ rosdep update
   $ rosdep check --from-paths . --ignore-src --rosdistro kinetic
   ```
   请将 'kinetic' 修改为您在使用的ROS版本。如有任何未安装的依赖包列出，请执行以下命令自动安装:  
   ```bash
   $ rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
   ```
   同样的，请将 'kinetic' 修改为您在使用的ROS版本。  

## 4.4 编译代码
   ```bash
   $ cd ~/catkin_ws
   $ catkin_make
   ```
## 4.5 执行配置脚本
```bash
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```
如果在您的 ~/.bashrc中已经有了以上语句，直接运行:
```bash
$ source ~/.bashrc
```
## 4.6 在RViz环境试用:
```bash
$ roslaunch xarm_description xarm7_rviz_display.launch
```
## 4.7 如果已安装Gazebo,可以执行demo查看效果
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch [run_demo:=true] [add_gripper:=true] [add_vacuum_gripper:=true] 
   ```
&ensp;&ensp;指定'run_demo'为true时Gazebo环境启动后机械臂会自动执行一套编好的循环动作。 这套简单的command trajectory写在xarm_controller\src\sample_motion.cpp. 这个demo加载的控制器使用position interface（纯位置控制）。  
&ensp;&ensp;指定'add_gripper'为true时, 会加载带有xarm 夹爪的模型。  
&ensp;&ensp;指定'add_vacuum_gripper'为true时, 会加载带有xarm 真空吸头的模型。注意：只能加载一款末端器件。  

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
请注意: xarm_moveit_config相关package会将所有关节限制在`[-pi, pi]`范围内, 因为如果不加限制，moveit可能会解出关节运动范围很大的轨迹。这个关节范围限制可以通过设置`...moveit_config/launch/planning_context.launch`文件中的"limited:=false"来取消。  

对于任何需要将**运动学校准**参数加入URDF模型的型号, 请使用[uf_robot_moveit_config](uf_robot_moveit_config/Readme_cn.md), 这是一个**统一的**集合了全部UFACTORY机械臂型号的`moveit_config`包，并且支持一些新特性。   

&ensp;&ensp;
   部分文档由moveit_setup_assistant自动生成, 用于Moveit Planner和Rviz可视化仿真。如果已安装MoveIt!,可以尝试跑demo： 
   ```bash
   $ roslaunch xarm7_moveit_config demo.launch
   ```

#### Moveit!图形控制界面 + Gazebo 仿真环境:  
   1. 如果不需要带有xArm Gripper，首先执行:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch
   ```
   然后在另一个终端运行:
   ```bash
   $ roslaunch xarm7_moveit_config xarm7_moveit_gazebo.launch
   ```
   2. 如果**需要带有xArm Gripper**，首先执行:  
   ```bash
   $ roslaunch xarm_gazebo xarm7_beside_table.launch add_gripper:=true
   ```
   然后在另一个终端运行:
   ```bash
   $ roslaunch xarm7_gripper_moveit_config xarm7_gripper_moveit_gazebo.launch
   ```
   如果您在Moveit界面中规划了一条满意的轨迹, 点按"Execute"会使Gazebo中的虚拟机械臂同步执行此轨迹。  

   3. 如果**需要带有xArm 真空吸头**，用"vacuum_gripper"替换掉上面例子中的"gripper"关键字即可。  

#### Moveit!图形控制界面 + xArm 真实机械臂:
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_moveit_config realMove_exec.launch robot_ip:=<控制盒的局域网IP地址> [velocity_control:=false] [report_type:=normal]
   ```
   检查terminal中的输出看看有无错误信息。如果启动无误，您可以将RViz中通过Moveit规划好的轨迹通过'Execute'按钮下发给机械臂执行。***但一定确保它不会与周围环境发生碰撞！***   
   `velocity_control`为可选参数, 如果设置为`true`, velocity controller 和 velocity interface 就会取代默认的位置控制接口； `report_type`同样为可选参数, 具体请参考[这里](#report_type-启动参数).  

#### Moveit!图形控制界面 + 安装了UFACTORY机械爪的xArm真实机械臂:  
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   如果使用了我们配套的机械爪(xArm gripper), 最好可以使用这个package，因为其中的配置会让Moveit在规划无碰撞轨迹时将机械爪考虑在内。 

#### Moveit!图形控制界面 + 安装了UFACTORY真空吸头的xArm真实机械臂:  
   首先, 检查并确认xArm电源和控制器已上电开启, 然后运行:  
   ```bash
   $ roslaunch xarm7_vacuum_gripper_moveit_config realMove_exec.launch robot_ip:=<your controller box LAN IP address>
   ```
   如果使用了我们配套的真空吸头(xArm vacuum gripper), 最好可以使用这个package，因为其中的配置会让Moveit在规划无碰撞轨迹时将真空吸头考虑在内。 

## 5.5.1 Moveit加载其它自定义模型到机械臂末端
&ensp;&ensp;***此部分功能需要使用ROS Melodic或之后的版本***  
&ensp;&ensp;在 __xarm5_moveit_config__/__xarm6_moveit_config__/__xarm7_moveit_config__ 这三个包里，可通过以下参数加载其它工具模型到机械臂末端，以实现在moveit规划中加入自定义末端工具的偏移量和碰撞检测。(注意：使用/xarm/set_tcp_offset服务的设置对moveit规划不生效！) 

### 使用示例
   ```bash
   # 加载box模型
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=box

   # 加载cylinder模型
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=cylinder

   # 加载sphere模型
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=sphere

   # 加载其它mesh模型（这里加载vacuum_gripper为例，如果加载的模型是放在xarm_description/meshes/other里面，geometry_mesh_filename参数只需要传文件名）
   $ roslaunch xarm7_moveit_config demo.launch add_other_geometry:=true geometry_type:=mesh geometry_mesh_filename:=package://xarm_description/meshes/vacuum_gripper/xarm/visual/vacuum_gripper.stl geometry_mesh_tcp_xyz:='"0 0 0.126"'
   ```

### 参数说明
- __add_other_geometry__: 默认为false，表示是否加载其它几何模型到末端
- __geometry_type__: 要加载的几何模型的类型，支持box/cylinder/sphere/mesh这几种，不同种类支持的参数不一样
- __geometry_height__: 几何模型高度，单位(米)，默认0.1，仅在geometry_type为box/cylinder/sphere时有效
- __geometry_radius__: 几何模型半径，单位(米)，默认0.1，仅在geometry_type为cylinder/sphere时有效
- __geometry_length__: 几何模型长度，单位(米)，默认0.1，仅在geometry_type为box时有效
- __geometry_width__: 几何模型宽度，单位(米)，默认0.1，仅在geometry_type为box时有效
- __geometry_mesh_filename__: 几何模型的文件名，仅在geometry_type为mesh时有效
- __geometry_mesh_origin_xyz__: 几何模型的参考系相对于末端的参考系, 默认"0 0 0"，仅在geometry_type为mesh时有效
- __geometry_mesh_origin_rpy__: 几何模型的参考系相对于末端的参考系, 默认"0 0 0"，仅在geometry_type为mesh时有效
- __geometry_mesh_tcp_xyz__: 几何模型相对于末端的偏移, 默认"0 0 0"，仅在geometry_type为mesh时有效
- __geometry_mesh_tcp_rpy__: 几何模型相对于末端的偏移, 默认"0 0 0"，仅在geometry_type为mesh时有效  

## 5.6 xarm_planner:
这个简单包装实现的规划器接口是基于 Moveit!中的 move_group interface, 可以使用户通过service指定目标位置进行规划和执行。 这部分的详细使用方法请阅读[xarm_planner包](./xarm_planner/ReadMe_cn.md)的文档。  
#### 启动 xarm simple motion planner 控制 xArm 真实机械臂:  
```bash
   $ roslaunch xarm_planner xarm_planner_realHW.launch robot_ip:=<控制盒的局域网IP地址> robot_dof:=<7|6|5>
```
'robot_dof'参数指的是xArm的关节数目 (默认值为7)。xarm_planner已经可以支持装载UF机械爪或真空吸头的xArm模型，请根据需要指定"**add_gripper**"或"**add_vacuum_gripper**"为true。   

## 5.7 xarm_api/xarm_msgs:
&ensp;&ensp;这两个package提供给用户封装了xArm SDK功能的ros服务, xarm自带的控制盒会进行轨迹规划。当前支持12种运动命令（ros service同名）, 请首先务必确保手臂工作在正确的模式下, 参考[模式切换](#6-模式切换):  
#### 手臂模式0:
* `move_joint`: 关节空间的点到点运动, 用户仅需要给定目标关节位置，运动过程最大关节速度/加速度即可， 对应SDK里的set_servo_angle()函数。 
* `move_line`: 笛卡尔空间的直线轨迹运动，用户需要给定工具中心点（TCP）目标位置以及笛卡尔速度、加速度，对应SDK里的set_position()函数【不指定交融半径】。  
* `move_lineb`: 笛卡尔空间的直线轨迹运动, 同时与下一条运动指令做交融连续。可以准备一系列已知的中间点以及目标位置。 每两个中间点间为直线轨迹，但在中间点处做一个圆弧过渡（需给定半径）来保证速度连续，对应SDK里的set_position()函数【指定了交融半径和wait=false】。代码示例请参考[move_test.cpp](./xarm_api/test/move_test.cpp) 和 [blended_motion_test.py](./xarm_api/scripts/blended_motion_test.py)，为了提前进行交融运算，`/xarm/wait_for_finish` 参数必须设置为`false`。    
* `move_jointb`: 关节空间的点到点运动, 同时与下一条运动指令做交融连续。可以与"move_lineb"混用实现关节和线性运动的交融，只要中间点位已知且交融半径正确设置, 速度将在执行过程中保持连续而不停顿。对应SDK里的set_servo_angle()函数【指定了交融半径和wait=false】。代码示例请参考[blended_motion_test.py](./xarm_api/scripts/blended_motion_test.py)，为了提前进行交融运算，`/xarm/wait_for_finish` 参数必须设置为`false`。    
* `move_line_tool`: 基于工具坐标系（而不是基坐标系）的直线运动。对应SDK里的set_tool_position()函数。  
另外需要 ***注意*** 的是，使用以上4种service之前，需要通过service依次将机械臂模式(mode)设置为0，然后状态(state)设置为0。这些运动指令的意义和详情可以参考产品使用指南。除此之外还提供了其他xarm编程API支持的service调用, 对于相关ros service的定义在 [xarm_msgs目录](./xarm_msgs/)中。  
* `move_line_aa`: 笛卡尔空间的直线轨迹运动，姿态使用**轴-角** 而不是roll-pitch-yaw欧拉角，在使用此命令之前请仔细查阅xArm用户手册关于轴-角的解释。  

#### 手臂模式1:
* `move_servo_cart/move_servoj`: （固定）高频率的笛卡尔或关节轨迹指令，分别对应SDK里的set_servo_cartesian()和set_servo_angle_j()，需要机械臂工作在**模式1**，可以间接实现速度控制。在使用这两个服务功能之前，务必做好**风险评估**并且仔细阅读[其他示例 2-3节](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#2-servo_cartesian-streamed-cartesian-trajectory)的使用方法。    

#### 手臂模式4:
* `velo_move_joint/velo_move_joint_timed`: 指定所有关节目标转速(单位: rad/s)的运动, 最大关节加速度可以通过 `set_max_acc_joint` 服务设定。（[例子](#5-关节旋转速度控制)）  

#### 手臂模式5:
* `velo_move_line/velo_move_line_timed:` 指定TCP笛卡尔线速度（mm/s）和姿态角速度（rad/s，**轴-角速度**表示）的运动, 最大关节加速度可以通过`set_max_acc_line` 服务设定。（[例子](#6-笛卡尔线速度控制)）  

#### 手臂模式6: 
* `move_joint`: 关节在线轨迹规划（需要**控制器固件版本 >= v1.10.0**），在模式6下，可以随时发送新的关节目标位置，最大速度和加速度，控制器可以在线重新规划轨迹。目标更改后关节速度加速度连续，但可能不再同步加减速，最终到达的位置可能会稍有误差。**本功能主要是实现关节指令的动态响应，而不像servo_joint指令需要用户自己规划轨迹并高频更新**。 `/xarm/wait_for_finish` 需要设置为 `false` 以随时打断现有目标轨迹，对应的SDK函数仍然是"set_servo_angle(wait=false)"。（[例子](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)）  

#### 手臂模式7:
* `move_line`： 笛卡尔空间路径在线规划（需要**控制器固件版本 >= v1.11.0**），在模式7下，可以随时更新目标笛卡尔坐标，以及期望的最大速度和加速度，控制器可以及时在线重新规划路径，以线速度连续的方式过渡到新的目标位置。 **本功能主要是实现笛卡尔指令的动态响应，而不像servo_cartesian指令需要用户自己规划轨迹并高频更新**. `/xarm/wait_for_finish` 需要设置为 `false` 以随时打断现有目标轨迹，对应的SDK函数仍然是"set_position(wait=false)"。（[例子](https://github.com/xArm-Developer/xarm_ros/tree/master/examples#6-online-target-update)）  

#### 使用ROS Service启动 xArm:

&ensp;&ensp;首先启动xarm7 service server, 以下ip地址只是举例:  
```bash
$ roslaunch xarm_bringup xarm7_server.launch robot_ip:=192.168.1.128 report_type:=normal
```
`report_type`为可选参数，参考[这里](#report_type-启动参数)  

&ensp;&ensp;然后确保每个关节的控制已经使能, 参考[SetAxis.srv](/xarm_msgs/srv/SetAxis.srv):
```bash
$ rosservice call /xarm/motion_ctrl 8 1
```
&ensp;&ensp;在传递任何运动指令前，先***依次***设置正确的机械臂模式(0: POSE)和状态(0: READY), 参考[SetInt16.srv](/xarm_msgs/srv/SetInt16.srv):    
```bash
$ rosservice call /xarm/set_mode 0

$ rosservice call /xarm/set_state 0
```

#### 关节空间和笛卡尔空间运动指令的示例:
&ensp;&ensp;请注意角度应全部使用**radian**作为单位。以下运动命令都使用同类型的srv request: [Move.srv](./xarm_msgs/srv/Move.srv)。  
##### 1. 关节空间运动:
&ensp;&ensp;调用关节运动命令，最大速度 0.35 rad/s，加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/move_joint [0,0,0,0,0,0,0] 0.35 7 0 0
```
&ensp;&ensp;调用回原点服务 (各关节回到0角度)，最大角速度 0.35 rad/s，角加速度 7 rad/s^2:  
```bash
$ rosservice call /xarm/go_home [] 0.35 7 0 0
```
##### 2. 基坐标系笛卡尔空间运动:
&ensp;&ensp;调用笛卡尔空间指令，目标位置表示在机械臂基坐标系中，最大线速度 200 mm/s，加速度为 2000 mm/s^2:  
```bash
$ rosservice call /xarm/move_line [250,100,300,3.14,0,0] 200 2000 0 0
```
##### 3. 工具坐标系笛卡尔空间运动:
&ensp;&ensp;调用笛卡尔空间指令，目标位置表示在机械臂当前工具坐标系中，最大线速度 200 mm/s，加速度为 2000 mm/s^2，以下指令将基于当前工具坐标系做**相对移动**(delta_x=50mm, delta_y=100mm, delta_z=100mm), 没有姿态的相对变化：  
```bash
$ rosservice call /xarm/move_line_tool [50,100,100,0,0,0] 200 2000 0 0
```
##### 4. 轴-角姿态表示的笛卡尔空间运动:
&ensp;&ensp;轴-角笛卡尔运动对应的服务是[MoveAxisAngle.srv](./xarm_msgs/srv/MoveAxisAngle.srv)。请仔细阅读并留意最后两个参数: "**coord**" 为0代表在手臂基坐标系中运动, 为1代表在末端坐标系中运动。"**relative**" 为0代表给定的目标为指定坐标系下的绝对位置，为1代表给定目标为一个相对位置。 
&ensp;&ensp;例如: 围绕当前工具坐标系的Z轴旋转 1.0 弧度: 
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
或者：
```bash
$ rosservice call /xarm/move_line_aa [0,0,0,0,0,1.0] 30.0 100.0 0.0 1 1
```   
&ensp;&ensp;"**mvtime**" 在此命令中无意义，设为0即可。再比如: 在基坐标系下, 沿Y轴方向平移122mm, 同时绕X轴旋转-0.5弧度:  
```bash
$ rosservice call /xarm/move_line_aa [0,122,0,-0.5,0,0] 30.0 100.0 0.0 0 1  
```

##### 5. 关节旋转速度控制:
&ensp;&ensp;（需要**控制器固件版本 >= 1.6.8**）如果需要对关节速度进行控制, 请先根据[模式切换](#6-模式切换)设置为**Mode 4**, 使用前请阅读[MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv)并理解速度控制服务输入参数的含义。对于关节速度控制, 如果多于一个关节参与运动，可以设置**jnt_sync** 参数为 1 来确保每个运动关节同步进行加、减速，否则每个关节会以自己最快的方式达到指定速度。***coord*** 参数在这里没有意义，设为0即可。例如: 
```bash
# 无超时版本： (手臂在收到全零指令前不会停止运动!)
$ rosservice call /xarm/velo_move_joint [0.1,-0.1,0,0,0,-0.3] 1 0
# 带超时版本(控制器固件版本 >= 1.8.0)：(如果下一条速度指令没有在0.2秒以内收到, xArm将会停止)
$ rosservice call /xarm/velo_move_joint_timed [0.1,-0.1,0,0,0,-0.3] 1 0 0.2
``` 
会命令对应的关节(for xArm6)以对应的速度(in rad/s)运动，并且所有关节会同步达到指定速度。最大关节加速度可以用以下方法设置，单位rad/s^2:  
```bash
$ rosservice call /xarm/set_max_acc_joint 10.0  (maximum: 20.0 rad/s^2)
``` 

##### 6. 笛卡尔线速度控制:
&ensp;&ensp;（需要**控制器固件版本 >= 1.6.8**）如果期望控制TCP朝着某一方向的线速度, 请先根据[模式切换](#6-模式切换)设置为**Mode 5**, 使用前请阅读[MoveVelo.srv](./xarm_msgs/srv/MoveVelo.srv)并理解速度控制服务输入参数的含义。如果指定的线速度是对应用户/基坐标系，设置 **coord** 参数为 0，如果是对应工具坐标系则设置为 1。***jnt_sync*** 参数在这里无意义，设置为0即可。例如: 
```bash
# 无超时版本： (手臂在收到全零指令前不会停止运动!)
$ rosservice call /xarm/velo_move_line [30,0,0,0,0,0] 0 1
# 带超时版本(控制器固件版本 >= 1.8.0)：(如果下一条速度指令没有在0.2秒以内收到, xArm将会停止)
$ rosservice call /xarm/velo_move_line_timed [30,0,0,0,0,0] 0 1 0.2
``` 
会指定xArm的末端TCP沿着工具坐标系X轴以30 mm/s的速度移动，最大线性加速度可以用以下方法设置，单位mm/s^2:  
```bash
$ rosservice call /xarm/set_max_acc_line 5000.0  (maximum: 50000 mm/s^2)
``` 
对于姿态旋转速度控制, 请留意姿态速度是以**轴-角速度**方式指定，即：[(指定坐标系下)单位旋转矢量] 乘以 [旋转角速度标量]。例如, 
```bash
# 无超时版本： (手臂在收到全零指令前不会停止运动!)
$ rosservice call /xarm/velo_move_line [0,0,0,0.707,0,0] 0 0
# 带超时版本(控制器固件版本 >= 1.8.0)：(如果下一条速度指令没有在0.2秒以内收到, xArm将会停止)
$ rosservice call /xarm/velo_move_line_timed [0,0,0,0.707,0,0] 0 0 0.2
``` 
会命令末端TCP绕着用户/基坐标系的X轴以45度/秒速度旋转，姿态变化的最大加速度目前是一个固定值。  

请注意: 速度模式下，无超时版本服务可以通过两种方式停止： 发送**全零速度**指令, 或者设置**state为4(STOP)** 然后下一次运动前再设置回0(READY)。但还是**推荐使用带超时的版本（固件更新至v1.8.0以上）**，这样在程序异常或通信中断等情况下可以保证安全。     


##### 运动服务返回值:
&ensp;&ensp;请注意以上的运动服务调用在默认情况下会**立刻返回**，如果希望等待运动结束之后再返回, 需要提前设置 ros parameter **"/xarm/wait_for_finish"** 为 **true**. 即:  
```bash
$ rosparam set /xarm/wait_for_finish true
```   
&ensp;&ensp;如果成功会返回 0，发生错误则会返回1.  

#### 工具 I/O 操作:
&ensp;&ensp;我们在机械臂末端提供了两路数字、两路模拟输入信号接口，以及两路数字输出信号接口。  
##### 1. 同时获得2个数字输入信号状态的方法:  
```bash
$ rosservice call /xarm/get_digital_in
```
##### 2. 获得某一模拟输入信号状态的方法: 
```bash
$ rosservice call /xarm/get_analog_in 1  (最后一个参数：端口号，只能是1或者2)
```
##### 3. 设定某一个输出端口电平的方法:
```bash
$ rosservice call /xarm/set_digital_out 2 1  (设定输出端口2的逻辑为1)
```
&ensp;&ensp;注意检查这些service返回的"ret"值为0，来确保操作成功。

#### 控制器 I/O 操作:
&ensp;&ensp;我们在控制盒外侧提供了8/16路数字输入和8/16路数字输出信号接口。  

##### 1. 获得某一数字输入信号状态的方法: 
```bash
$ $ rosservice call /xarm/get_controller_din io_num (注意：从1到8, 对应CI0到CI7，9-16对应DI0到DI7[如有])  
```
##### 2. 设定某一个输出端口电平的方法:
```bash
$ rosservice /xarm/set_controller_dout io_num (注意：从1到8, 对应CO0到CO7，9-16对应DI0到DI7[如有]) logic (0或1) 
```
&ensp;&ensp;例如:  
```bash
$ rosservice call /xarm/set_controller_dout 5 1  (设定输出端口5[标签CO4]的逻辑为1)
```
##### 3. 读取某一端口模拟输入量的方法:
```bash
$ rosservice call /xarm/get_controller_ain port_num  (注意: 从1到2, 对应 AI0~AI1)
```
##### 4. 设定某一端口模拟输出量的方法:
```bash
$ rosservice call /xarm/set_controller_aout port_num (注意: 从1到2, 对应 AO0~AO1) analog_value 
```
&ensp;&ensp;例如:  
```bash
$ rosservice call /xarm/set_controller_aout 2 3.3  (设定输出端口 AO1 为 3.3)
```
&ensp;&ensp;注意检查这些service返回的"ret"值为0，来确保操作成功。


#### 获得反馈状态信息:
&ensp;&ensp;如果通过运行'xarm7_server.launch'连接了一台xArm机械臂，用户可以通过订阅 ***"xarm/xarm_states"*** topic 获得机械臂当前的各种状态信息， 包括关节角度、工具坐标点的位置、错误、警告信息等等。具体的信息列表请参考[RobotMsg.msg](./xarm_msgs/msg/RobotMsg.msg).  
&ensp;&ensp;另一种选择是订阅 ***"/joint_states"*** topic, 它是以[JointState.msg](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html)格式发布数据的, 但是当前 ***只有 "position" 是有效数据***; "velocity" 是没有经过任何滤波的基于相邻两组位置数据进行的数值微分, "effort" 的反馈数据是基于电流的估计值，而不是直接从力矩传感器获得，因而它们只能作为参考。
&ensp;&ensp;基于运行时性能考虑，目前以上两个topic的数据更新率固定为 ***5Hz***。状态反馈的频率和内容可以有其他选择，参考[report_type参数](#report_type-启动参数)。  

#### 关于设定末端工具偏移量(适用于xarm_api service控制):  
&ensp;&ensp;末端工具的偏移量可以也通过'/xarm/set_tcp_offset'服务来设定,参考下图，请注意这一坐标偏移量是基于 ***默认工具坐标系*** (坐标系B)描述的，它位于末端法兰中心，并且相对基坐标系(坐标系A)有（PI, 0, 0)的RPY旋转偏移。
![xArmFrames](./doc/xArmFrames.png)  
&ensp;&ensp;例如：
```bash
$ rosservice call /xarm/set_tcp_offset 0 0 20 0 0 0
```
&ensp;&ensp;这条命令设置了基于原始工具坐标系(x = 0 mm, y = 0 mm, z = 20 mm)的位置偏移量，还有（0 rad, 0 rad, 0 rad)的RPY姿态偏移量。***请注意：这里设置的TCP偏移在后续xArm Studio的操作中可能被重置（如果这个设定和studio中的默认设置不同）*** 如果需要xArm Studio和ros service配合控制机械臂，建议在xArm Studio中做好相同的默认TCP偏移设置。 

#### 清除错误:
&ensp;&ensp;有时控制器会因为掉电、位置或速度超限、规划失败等原因汇报错误，遇到这一状态需要手动解除。具体的错误代码可以在topic ***"xarm/xarm_states"*** 的信息中找到。 
```bash
$ rostopic echo /xarm/xarm_states
```
&ensp;&ensp;如果'err'字段数据为非零，需要对照用户手册找到原因并设法解决问题。之后，这一错误状态可以通过调用服务 ***"/xarm/clear_err"*** 清除：  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;如果正在使用 Moveit!, 可以调用 "**/xarm/moveit_clear_err**", 这样可以省去再次手动设置模式1.   
```bash
$ rosservice call /xarm/moveit_clear_err
```
&ensp;&ensp;调用此服务之后 ***务必再次确认err状态信息*** , 如果它变成了0, 说明问题清除成功，否则请再次确认问题是否成功解决。清除成功之后， 记得 ***将robot state设置为0*** 以便使机械臂可以执行后续指令。  

#### 机械爪控制:
&ensp;&ensp;如果已将xArm Gripper (UFACTORY出品) 安装至机械臂末端，则可以使用如下的service或action来操作和检视机械爪:  

##### 1. Gripper services:  
(1) 首先使能xArm机械爪并设置抓取速度, 如果成功，'ret'返回值为0。正确的速度范围是在***1到5000之间***，以1500为例:  
```bash
$ rosservice call /xarm/gripper_config 1500
```
(2) 给定xArm机械爪位置（打开幅度）指令执行，如果成功，'ret'返回值为0。正确的位置范围是***0到850之间***, 0为关闭，850为完全打开，以500为例:  
```bash
$ rosservice call /xarm/gripper_move 500
```
(3) 获取当前xArm机械爪的状态（位置和错误码）:
```bash
$ rosservice call /xarm/gripper_state
```
&ensp;&ensp;如果错误码不为0，请参考使用说明书查询错误原因，清除错误同样可使用上一节的clear_err service。  

##### 2. Gripper action:
&ensp;&ensp; gripper move action 定义在 [Move.action](/xarm_gripper/action/Move.action). 目标包括期望的机械爪脉冲位置和脉冲速度. 可以通过设置xarm_bringup/launch/xarm7_server.launch 文件中的 "**use_gripper_action**" 参数来启动 action 服务器. Gripper action 可以这样调用:  
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
&ensp;&ensp; 或者通过以下方法调用:
```bash
$ rosrun xarm_gripper gripper_client 500 1500 
```

#### 真空吸头控制:
&ensp;&ensp; 如果已将真空吸头 (UFACTORY出品) 安装至机械臂末端，则可以使用如下的service来操作真空吸头。  

&ensp;&ensp;吸头开启:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 1
```
&ensp;&ensp;吸头关闭:  
```bash
$ rosservice call /xarm/vacuum_gripper_set 0
```
&ensp;&ensp;正常执行服务将返回0.  


#### 末端工具Modbus通信:
如果需要与末端工具进行modbus通讯, 需要先通过"xarm/config_tool_modbus"服务设置正确的通信波特率和超时时间 (ms) (参考 [ConfigToolModbus.srv](/xarm_msgs/srv/ConfigToolModbus.srv)). 例如: 
```bash
$ rosservice call /xarm/config_tool_modbus 115200 20
```
以上命令会设置末端modbus通信波特率为 115200 bps，接收超时时间为 20**毫秒**。 如果之后这些参数没有改变就不需要重新设置。 **请注意** 设置一个新的波特率可能会返回1（报错误码28）而不是0, 实际上如果设备已正确连接且没有其他导致通信错误的因素，设置是已经正确执行的。您可以清除错误后重新调用一次这个服务确定是否返回0。当前仅支持如下波特率设置 (单位bps): [4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1000000, 1500000, 2000000, 2500000].  

设置完成后，modbus通信可以这样通过rosservice进行 (参考 [SetToolModbus.srv](/xarm_msgs/srv/SetToolModbus.srv)):  
```bash
$ rosservice call /xarm/set_tool_modbus [0x01,0x06,0x00,0x0A,0x00,0x03] 6
```
第一个参数是要发送的通讯字节序列, 第二个参数是需要接收的回复字节数量. **这个数字应该是期望收到的数据字节数 (不含CRC校验字符)**. 举例来说，对于某个测试设备，上面的指令可能返回:  
```bash
ret: 0
respond_data: [1, 6, 0, 10, 0, 3]
```
其中实际收到的数据帧为: [0x01, 0x06, 0x00, 0x0A, 0x00, 0x03]，长度为6.  

#### "report_type" 启动参数:
当启动xArm真机的ROS应用时, 可以选择性的指定"report_type"参数。它决定了状态上报的内容和频率。具体请参考[开发者手册](https://www.cn.ufactory.cc/_files/ugd/896670_8c25a14281ce4b63814c1730c3fd82c4.pdf)中的**2.1.6. 自动上报数据格式**章节查看三种上报类型(`normal/rich/dev`)的内容区别, 默认使用的类型为 "normal"。  

* 对于需要高频率状态反馈的用户, 在启动时可以指定`report_type:=dev`, 这样`/xarm/xarm_states`和`/xarm/joint_states`话题会以**100Hz**频率更新。  
* 对于需要使用`/xarm/controller_gpio_states`话题实时查看控制器GPIO状态的用户, 请指定`report_type:=rich`, 可以在开发者手册中看到这个类型反馈的信息是最全的。  
* 不同上报类型的更新频率:   

|   type   |    port No.   | Frequency |  GPIO topic  | F/T sensor topic | 
|:--------:|:-------------:|:---------:|:------------:|:----------------:|
|   normal |     30001     |    5Hz    |     不可用    |      不可用        |
|   rich   |     30002     |    5Hz    |      可用     |       可用        | 
|   dev    |     30003     |    100Hz  |     不可用    |        可用        |

注: **GPIO topic** => `xarm/controller_gpio_states`. **F/T sensor topic** =>  `xarm/uf_ftsensor_ext_states` and `xarm/uf_ftsensor_raw_states`。

## 5.8 xarm_moveit_servo:
&ensp;&ensp;此模块用于通过特定外部输入设备来控制机械臂
   - #### 5.8.1 通过 __XBOX360__ 手柄控制
      - 左摇杆控制TCP的X和Y
      - 右摇杆控制TCP的ROLL和PITCH
      - [前面]左右两个触发器控制TCP的Z
      - [前面]左右两个缓冲器控制TCP的YAW
      - 十字键控制关节1和关节2的转动
      - 按键X和按键B控制最后一个关节的转动
      - 按键Y和按键A控制倒数第二个关节的转动

      ```bash
      # 控制真实xArm6机械臂
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=1
      # XBOX Wired -> joystick_type=1
      # XBOX Wireless -> joystick_type=2

      # 或者控制真实Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=1 robot_type:=lite
      ```

   - #### 5.8.2 通过六维鼠标 __3Dconnexion SpaceMouse Wireless__ 来控制
      - 六维鼠标的六个维度对应控制TCP的X/Y/Z/ROLL/PITCH/YAW
      - 左边按键按下时单独控制TCP的XYZ
      - 右边按键按下时单独控制TCP的ROLL/PITCH/YAW

      ```bash
      # 控制真实xArm6机械臂
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=3

      # 或者控制真实Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=3 robot_type:=lite
      ```

   - #### 5.8.3 通过 __键盘输入__ 控制
      ```bash
      # 控制真实xArm6机械臂
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.206 dof:=6 joystick_type:=99

      # 或者控制真实Lite6
      $ roslaunch xarm_moveit_servo xarm_moveit_servo_realmove.launch robot_ip:=192.168.1.52 dof:=6 joystick_type:=99 robot_type:=lite
      ```


# 6. 模式切换
&ensp;&ensp;xArm 在不同的控制方式下可能会工作在不同的模式中，当前的模式可以通过topic "xarm/xarm_states" 的内容查看。在某些情况下，需要用户主动切换模式以达到继续正常工作的目的。

### 6.1 模式介绍

&ensp;&ensp; ***Mode 0*** : 基于xArm controller规划的位置模式；   
&ensp;&ensp; ***Mode 1*** : 基于外部轨迹规划器的位置模式；  
&ensp;&ensp; ***Mode 2*** : 自由拖动(零重力)模式；  
&ensp;&ensp; ***Mode 3*** : 保留；  
&ensp;&ensp; ***Mode 4*** : 关节速度控制模式；  
&ensp;&ensp; ***Mode 5*** : 笛卡尔速度控制模式；  
&ensp;&ensp; ***Mode 6*** : 关节在线规划模式；（控制器固件版本>=v1.10.0） 
&ensp;&ensp; ***Mode 7*** : 笛卡尔路径在线规划模式。(控制器固件版本>= v1.11.0)  

&ensp;&ensp;***Mode 0*** 是系统初始化的默认模式，当机械臂发生错误(碰撞、过载、超速等等),系统也会自动切换到模式0。并且对于[xarm_api](./xarm_api/)包和[SDK](https://github.com/xArm-Developer/xArm-Python-SDK)中提供的运动指令都要求xArm工作在模式0来执行。***Mode 1*** 是为了方便像 Moveit! 一样的第三方规划器绕过xArm控制器的规划去执行轨迹。 ***Mode 2*** 可以打开自由拖动模式, 机械臂会进入零重力状态方便拖动示教, 但需注意在进入模式2之前确保机械臂安装方式和负载均已正确设置。 ***Mode 4*** 可以直接给定关节期望速度来控制手臂。***Mode 5*** 可以给定末端笛卡尔线速度来控制手臂运动。***Mode 6 和 Mode 7***对应关节和笛卡尔在线规划模式，可以随时动态更新指令，控制器自动规划并执行到新的目标。

### 6.2 切换模式的正确方法:  
&ensp;&ensp;如果在执行Moveit!规划的轨迹期间发生碰撞等错误, 为了安全考虑，当前模式将被自动从1切换到0, 同时robot state将变为 4 (错误状态)。这时即使碰撞已经解除，机械臂在重新回到模式1之前也无法执行任何Moveit!或者servoj指令。请依次按照下列指示切换模式:  

&ensp;&ensp;(1) 确认引发碰撞的物体已经被移除；  
&ensp;&ensp;(2) 清除错误:  
```bash
$ rosservice call /xarm/clear_err
```
&ensp;&ensp;(3) 切换回想要的模式 (以Mode 2为例), 之后 set state 为0（Ready状态）:
```bash
$ rosservice call /xarm/set_mode 2

$ rosservice call /xarm/set_state 0
```

# 7. xArm视觉
提供xArm扩展视觉应用的基础示例，包括手眼标定和视觉抓取，例程主要基于[Intel RealSense D435i](https://www.intelrealsense.com/depth-camera-d435i/)深度相机。 

## 7.1 相关依赖库和ros包的安装：

首先进入ros工作空间：
```bash
$ cd ~/catkin_ws/src/
```

### 7.1.1 安装RealSense 开发支持库和ROS软件包： 
请依照[官方指示步骤](https://github.com/IntelRealSense/realsense-ros)正确安装。

### 7.1.2 安装aruco_ros, 用于手眼标定：
参考[官方Github](https://github.com/pal-robotics/aruco_ros):
```bash
$ git clone -b kinetic-devel https://github.com/pal-robotics/aruco_ros.git
```
### 7.1.3 安装easy_handeye, 用于手眼标定：
参考[官方Github](https://github.com/IFL-CAMP/easy_handeye):
```bash
$ git clone https://github.com/IFL-CAMP/easy_handeye
``` 
### 7.1.4 安装find_object_2d包，用于物体识别：
参考[官方Github](https://github.com/introlab/find-object/tree/kinetic-devel):
```bash
$ sudo apt-get install ros-kinetic-find-object-2d
```
### 7.1.5 安装其他依赖包：
```bash
$ cd ~/catkin_ws
```
然后请参考[4.3节内容](#43-安装其他依赖包).

### 7.1.6 编译整个工作区：
```bash
$ catkin_make
```

## 7.2 手眼标定示例：
如果使用RealSense D435i相机配合固定工件安装在手臂末端，即“**眼在手上**”，确保相机与电脑通信正常且手臂正常上电后，可以参考和使用如下launch脚本进行手眼标定：
```bash
$ roslaunch d435i_xarm_setup d435i_xarm_auto_calib.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
注意: 对于**2023年8月之后**生产的xArm/UF850系列型号, 可以选择将运动学校准参数加入到URDF模型中, 在以上的launch命令中使用`kinematics_suffix`参数来提高标定的准确度, 详情参考[这里](https://github.com/xArm-Developer/xarm_ros/blob/master/uf_robot_moveit_config/Readme_cn.md#optional-parameters)。   

标定使用的aruco二维码可以在[这里下载](https://chev.me/arucogen/)，请记住自己下载的`marker ID`和`marker size`，并在以上launch文件中修改。参考[官方](https://github.com/IFL-CAMP/easy_handeye#calibration)或其他网络教程通过图形界面进行标定，标定完成并确认保存后，默认会在 `~/.ros/easy_handeye`目录下生成`.yaml`后缀的结果文档，供后续与手臂一起进行坐标变换使用。如果固定件用的是UFACTORY提供的[camera_stand](https://www.ufactory.cc/products/xarm-camera-module-2020)，在xarm_vision/d435i_xarm_setup/config/[xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml](./xarm_vision/d435i_xarm_setup/config/xarm_realsense_handeyecalibration_eye_on_hand_sample_result.yaml)中保存了参考的标定结果。 

### 7.2.1 关于 UFACTORY Lite6 手眼标定:
请首先阅读和了解上面7.2章节关于xarm系列的标定示例，然后使用下面列出的替换文件应用于lite6的标定：  
Ufactory Lite6标定启动文件:
```bash
$ roslaunch d435i_xarm_setup d435i_lite6_auto_calib.launch robot_ip:=your_xArm_IP
```
标定结果文档示例: [lite6_realsense_handeyecalibration_eye_on_hand_sample_result.yaml](./xarm_vision/d435i_xarm_setup/config/lite6_realsense_handeyecalibration_eye_on_hand_sample_result.yaml)

标定结果发布启动文件示例:
[publish_handeye_tf_lite6.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf_lite6.launch)

### 7.2.2 手眼标定注意事项:
由于`easy_handeye`默认生成的机械臂位置的位姿变化不大，导致最终的标定结果可能不是太准确也不那么稳定。
在实际标定过程中我们可以不使用`easy_handeye`生成的位置，可以在上述命令指定启动参数`freehand_robot_movement:=true`启动，通过xarm studio控制界面或者开启拖动示教认为控制机械臂到不同位置，然后通过启动的手眼标定窗口的"__Take Sample__"采集数据，采集到大概17个数据后通过"__Compute__"计算，每次采集时机械臂的位置建议在保证标定板在视野范围内尽量多旋转rpy
- 两次运动的旋转轴的夹角越大越好
- 每次运动的旋转矩阵对应的旋转角度越大越好
- 相机中心到标定板的距离越小越好
- 每次运动机械臂末端运动的距离越小越好

## 7.3 3D视觉抓取示例：
本部分提供利用[***find_object_2d***](http://introlab.github.io/find-object/)进行简单的物体识别和抓取的示例程序。使用了RealSense D435i深度相机，UFACTORY camera_stand以及xArm官方机械爪。  

1.使用moveit驱动手臂动作，如果规划成功会保证无碰撞和奇异点的轨迹执行, 但对网络通信稳定性要求较高：
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_moveit_planner.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
如果目标物体可以正常识别，执行抓取节点:  
```bash
$ rosrun d435i_xarm_setup findobj2d_grasp_moveit
```
请注意在其中包含的[publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch)中，默认使用前面提到的参考标定结果，将识别的物体从相机坐标映射到机械臂基坐标系，可以按照需要改为其他标定结果yaml文件。节点代码可以参考d435i_xarm_setup/src/[findobj_grasp_moveit_planner.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_planner.cpp).  

2.或者使用xarm_api提供的ros service驱动手臂动作，网络稳定性要求不高，但部分时候执行过程中可能报错（奇异点或将要发生自碰撞等）：
```bash
$ roslaunch d435i_xarm_setup d435i_findobj2d_xarm_api.launch robot_dof:=your_xArm_DOF robot_ip:=your_xArm_IP
```
如果目标物体可以正常识别，执行抓取节点:  
```bash
$ roslaunch d435i_xarm_setup grasp_node_xarm_api.launch
```
请注意在其中包含的[publish_handeye_tf.launch](./xarm_vision/d435i_xarm_setup/launch/publish_handeye_tf.launch)中，默认使用前面提到的参考标定结果，将识别的物体从相机坐标映射到机械臂基坐标系，可以按照需要改为其他标定结果yaml文件。节点代码可以参考d435i_xarm_setup/src/[findobj_grasp_xarm_api.cpp](./xarm_vision/d435i_xarm_setup/src/findobj_grasp_moveit_xarm_api.cpp).  

***实际应用之前，请先读懂对应的代码，并针对自己的场景做出必要的修改***，比如抓取准备位置，姿态，抓取深度以及移动速度等等。代码中使用的识别目标名称为“object_1”，对应/objects目录下的`1.png`，用户可以根据实际应用在find_object_2d的图形界面中添加新的目标并修改节点程序中的`source_frame`，来识别感兴趣的物体。  

***Tips***: 应注意背景尽量干净而且颜色与被识别物体有区分度，如果目标物体有比较丰富的文理，识别率会更高。

## 7.4 在仿真的xArm模型末端添加RealSense D435i模型：
如果使用UFACTORY提供的camera stand固定，可以通过以下设置添加到虚拟模型（以xarm7为例）：  
```bash
 $ roslaunch xarm7_moveit_config demo.launch add_realsense_d435i:=true
```  

## 7.5 颜色块抓取例子

### 7.5.1 下载gazebo_grasp_plugin插件, 用于抓取仿真(melodic以上支持)
```bash
 # 进入ros命名空间src目录
 $ cd ~/catkin_ws/src/
 # 下载(针对不同ros版本请切换到对应分支)
 $ git clone https://github.com/JenniferBuehler/gazebo-pkgs.git
 $ git clone https://github.com/JenniferBuehler/general-message-pkgs.git
 # 编译
 $ cd ..
 $ catkin_make
```
### 7.5.2 Gazebo仿真(melodic以上支持)
```bash
 # 初始化gazebo场景和move_group
 $ roslaunch xarm_gazebo xarm_camera_scene.launch robot_dof:=6

 # 运行颜色块识别抓取脚本
 $ rosrun xarm_gazebo color_recognition.py
```
### 7.5.3 真机+realsense_d435i
```bash
 # 启动move_group
 $ roslaunch camera_demo xarm_move_group.launch robot_ip:=192.168.1.15 robot_dof:=6 add_realsense_d435i:=true

 # 运行颜色块识别抓取脚本(根据输出交互使用)
 $ rosrun camera_demo color_recognition.py
```

# 8. 其他示例
&ensp;&ensp;[在examples路径下](./examples)会陆续更新一些其他应用的demo例程，欢迎前去探索研究。
