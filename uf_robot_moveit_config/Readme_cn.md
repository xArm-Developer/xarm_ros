# UF ROBOT Moveit Config
- __此包用于通过moveit控制UFACTORY的各种机械臂__
- __支持 xArm5/xArm6/xArm7/Lite6/UFACTORY850 系列机械臂__
- __此包可以替代以下这些包的使用，启动脚本以及部分参数有区别__
  - xarm5_moveit_config/xarm5_gripper_moveit_config/xarm5_vacuum_moveit_config
  - xarm6_moveit_config/xarm6_gripper_moveit_config/xarm6_vacuum_moveit_config
  - xarm7_moveit_config/xarm7_gripper_moveit_config/xarm7_vacuum_moveit_config
  - lite6_moveit_config
- __此包并非通过moveit_setup_assistant生成，暂不支持使用moveit_setup_assistant修改__


## xArm 5
- #### Moveit!图形控制界面 + Rviz可视化仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm5_moveit_fake.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm5_moveit_fake.launch <可选参数>
  ```

- #### Moveit!图形控制界面 + 真实机械臂
  ```bash
  # 确保机械臂是通电以及急停开关是松开状态，并且网络是可以连通的
  
  roslaunch uf_robot_moveit_config xarm5_moveit_realmove.launch robot_ip:=<控制盒的局域网IP地址> <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm5_moveit_realmove.launch robot_ip_1:=<控制盒1的局域网IP地址> robot_ip_2:=<控制盒2的局域网IP地址> <可选参数>
  ```

- #### Moveit!图形控制界面 + Gazebo 仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm5_moveit_gazebo.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm5_moveit_gazebo.launch <可选参数>
  ```

## xArm 6
- #### Moveit!图形控制界面 + Rviz可视化仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm6_moveit_fake.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm6_moveit_fake.launch <可选参数>
  ```

- #### Moveit!图形控制界面 + 真实机械臂
  ```bash
  # 确保机械臂是通电以及急停开关是松开状态，并且网络是可以连通的

  roslaunch uf_robot_moveit_config xarm6_moveit_realmove.launch robot_ip:=<控制盒的局域网IP地址> <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm6_moveit_realmove.launch robot_ip_1:=<控制盒1的局域网IP地址> robot_ip_2:=<控制盒2的局域网IP地址> <可选参数>
  ```

- #### Moveit!图形控制界面 + Gazebo 仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm6_moveit_gazebo.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm6_moveit_gazebo.launch <可选参数>
  ```

## xArm 7
- #### Moveit!图形控制界面 + Rviz可视化仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm7_moveit_fake.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm7_moveit_fake.launch <可选参数>
  ```

- #### Moveit!图形控制界面 + 真实机械臂
  ```bash
  # 确保机械臂是通电以及急停开关是松开状态，并且网络是可以连通的

  roslaunch uf_robot_moveit_config xarm7_moveit_realmove.launch robot_ip:=<控制盒的局域网IP地址> <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm7_moveit_realmove.launch robot_ip_1:=<控制盒1的局域网IP地址> robot_ip_2:=<控制盒2的局域网IP地址> <可选参数>
  ```

- #### Moveit!图形控制界面 + Gazebo 仿真
  ```bash
  roslaunch uf_robot_moveit_config xarm7_moveit_gazebo.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_xarm7_moveit_gazebo.launch <可选参数>
  ```

## Lite 6
- #### Moveit!图形控制界面 + Rviz可视化仿真
  ```bash
  roslaunch uf_robot_moveit_config lite6_moveit_fake.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_lite6_moveit_fake.launch <可选参数>
  ```

- #### Moveit!图形控制界面 + 真实机械臂
  ```bash
  # 确保机械臂是通电以及急停开关是松开状态，并且网络是可以连通的

  roslaunch uf_robot_moveit_config lite6_moveit_realmove.launch robot_ip:=<控制盒的局域网IP地址> <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_lite6_moveit_realmove.launch robot_ip_1:=<控制盒1的局域网IP地址> robot_ip_2:=<控制盒2的局域网IP地址> <可选参数>
  ```

- #### Moveit!图形控制界面 + Gazebo 仿真
  ```bash
  roslaunch uf_robot_moveit_config lite6_moveit_gazebo.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_lite6_moveit_gazebo.launch <可选参数>
  ```

## UFACTORY 850
- #### Moveit!图形控制界面 + Rviz可视化仿真
  ```bash
  roslaunch uf_robot_moveit_config uf850_moveit_fake.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_uf850_moveit_fake.launch <可选参数>
  ```

- #### Moveit!图形控制界面 + 真实机械臂
  ```bash
  # 确保机械臂是通电以及急停开关是松开状态，并且网络是可以连通的

  roslaunch uf_robot_moveit_config uf850_moveit_realmove.launch robot_ip:=<控制盒的局域网IP地址> <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_uf850_moveit_realmove.launch robot_ip_1:=<控制盒1的局域网IP地址> robot_ip_2:=<控制盒2的局域网IP地址> <可选参数>
  ```

- #### Moveit!图形控制界面 + Gazebo 仿真
  ```bash
  roslaunch uf_robot_moveit_config uf850_moveit_gazebo.launch <可选参数>
  # 加载机械爪参数: add_gripper:=true
  # 加载吸泵参数: add_vacuum_gripper:=true
  # 使用速度控制: velocity_control:=true
  ```
  __双臂控制__
  ```bash
  roslaunch uf_robot_moveit_config dual_uf850_moveit_gazebo.launch <可选参数>
  ```

## Moveit! Planner
- ### 启动Planner控制节点
  - Moveit! Planner + Rviz可视化仿真
    ```bash
    roslaunch xarm_planner robot_planner_fake.launch robot_type:=xarm dof:=7 <可选参数>
    # robot_type: xarm/lite/uf850
    # dof: 
    #   robot_type=xarm: dof=5/6/7
    #   robot_type=lite/uf850: dof=6
    ```

  - Moveit! Planner + 真实机械臂
    ```bash
    roslaunch xarm_planner robot_planner_realmove.launch robot_ip:=<控制盒的局域网IP地址> robot_type:=xarm dof:=7 <可选参数>
    # robot_type: xarm/lite/uf850
    # dof: 
    #   robot_type=xarm: dof=5/6/7
    #   robot_type=lite/uf850: dof=6
    ```

- ### 调用Service进行规划 (这里以xarm7为例)
  - #### 关节空间点到点目标规划:  
    ```bash
    rosservice call xarm_joint_plan 'target: [1.0, -0.5, 0.0, -0.3, 0.0, 0.0, 0.5]'
    ```
    这种情况下列表中的元素代表每个关节的目标角度(单位是radian), 给定元素个数为关节数目。  

  - #### 笛卡尔空间点到点目标规划:  
    ```bash
    rosservice call xarm_pose_plan 'target: [[0.28, 0.2, 0.2], [1.0, 0.0, 0.0, 0.0]]'
    ```
    目标列表中的域分别指代工具坐标系原点位置(x, y, z)，单位：***米***；以及 ***四元数*** 方位(x, y, z, w)。注意此规划命令依然是简单的点到点运动，末端的执行轨迹并不是一条直线。  

  - #### 笛卡尔空间直线轨迹规划:
    ```bash
    rosservice call xarm_straight_plan 'target: [[0.28, 0.2, 0.2], [1.0, 0.0, 0.0, 0.0]]'
    ```
    指令数据的单位和之前的笛卡尔指令一致。如果规划成功，末端执行轨迹会是一条直线。但是在这种规划中，笛卡尔速度的变化可能不是很常规，请参考官方的Move Group Interface文档并在需要的时候修改代码。

- ### 执行规划好的轨迹:  
  ***请注意: 使用笛卡尔规划时, Moveit (OMPL) 每次生成的路径可能非常不同且比较随机，并不一定是移动距离最小的解, 所以在执行前一定在仿真环境中确认轨迹!*** 

  如果存在满意的解, 用户可以通过service(**推荐**)或topic下达执行命令。 

  - #### 通过service call执行 (阻塞式)
    调用'xarm_exec_plan' service 并将request的data设为'true', 则上一次成功解算的轨迹会被执行, service call会在执行完毕后返回，命令行运行:  
    ```bash
    rosservice call xarm_exec_plan 'true'
    ```

  - #### 通过topic执行 (非阻塞式)
    只需要通过话题"/xarm_planner_exec"发布一条消息 (类型: std_msgs/Bool), 内容设为'true'即可命令机械臂执行轨迹, 但是这种方法会直接返回而不是等待执行完成:  
    ```bash
    rostopic pub -1 /xarm_planner_exec std_msgs/Bool 'true'
    ```

## 可选参数
- ### 通用参数
  - __robot_sn__: 机械臂的SN，用于加载惯性参数，一般不需要指定，除非需要很精准的惯性参数
  - __model1300__: 是否是1300模型，仅对xarm系列有效，主要是末端模型有所变化，如果指定了`robot_sn`，会自动根据SN覆盖该参数
  - __limited__: 是否限定关节范围在(-180, 180)，默认为true
  - __attach_to__: 机械臂attach的连杆名，默认为world(此名默认会创建连杆)
  - __attach_xyz__: 机械臂相对`attach_to`的连杆的xyz偏移，默认为'0 0 0'
  - __attach_rpy__: 机械臂相对`attach_to`的连杆的rpy偏移, 默认为'0 0 0'
  - __add_realsense_d435i__: 是否加载RealSense D435i摄像头模型
    - __*add_d435i_links*__: 是否加载D435i的详细连杆，只在`add_realsense_d435i`为true有用
  - __add_gripper__: 是否加载机械爪, 默认为false，优先级高于`add_vacuum_gripper`和`add_other_geometry`
  - __add_vacuum_gripper__: 是否加载吸泵，默认为false，优先级低于`add_gripper`，但高于`add_other_geometry`
  - __add_other_geometry__: 是否添加其它几何模型到末端，默认为false，优先级低于`add_gripper`和`add_vacuum_gripper`
    - __*geometry_type*__: 要加载的几何模型的类型，支持box/cylinder/sphere/mesh这几种，不同种类支持的参数不一样
    - __*geometry_mass*__: 几何模型质量，单位(kg)，默认0.1
    - __*geometry_height*__: 几何模型高度，单位(米)，默认0.1，仅在geometry_type为box/cylinder/sphere时有效
    - __*geometry_radius*__: 几何模型半径，单位(米)，默认0.1，仅在geometry_type为cylinder/sphere时有效
    - __*geometry_length*__: 几何模型长度，单位(米)，默认0.1，仅在geometry_type为box时有效
    - __*geometry_width*__: 几何模型宽度，单位(米)，默认0.1，仅在geometry_type为box时有效
    - __*geometry_mesh_filename*__: 几何模型的文件名，geometry_type为mesh有效, 该文件需要存放于 *xarm_description/meshes/other/* 目录下面，这样就不需要在文件名里指定文件目录了
    - __*geometry_mesh_origin_xyz*__: 几何模型的基准参考系相对于末端法兰的参考系，geometry_type为mesh有效，使用时注意引号: geometry_mesh_origin_xyz:='"0.0 0.0 0.0"'
    - __*geometry_mesh_origin_rpy*__: 几何模型的基准参考系相对于末端法兰的参考系，geometry_type为mesh有效。使用时注意引号: geometry_mesh_origin_rpy:='"0.0 0.0 0.0"'
    - __*geometry_mesh_tcp_xyz*__: 几何模型末端(TCP)相对于几何模型基准参考系的偏移，geometry_type为mesh有效。使用时注意引号: geometry_mesh_tcp_xyz:='"0.0 0.0 0.0"'
    - __*geometry_mesh_tcp_rpy*__: 几何模型末端(TCP)相对于几何模型基准参考系的偏移，geometry_type为mesh有效。使用时注意引号: geometry_mesh_tcp_rpy:='"0.0 0.0 0.0"'
    
    ```bash
    # 加载box模型（此处以虚拟xArm7为例）
    roslaunch uf_robot_moveit_config xarm7_moveit_fake.launch add_other_geometry:=true geometry_type:=box

    # 加载cylinder模型（此处以虚拟xArm7为例）
    roslaunch uf_robot_moveit_config xarm7_moveit_fake.launch add_other_geometry:=true geometry_type:=cylinder

    # 加载sphere模型（此处以虚拟xArm7为例）
    roslaunch uf_robot_moveit_config xarm7_moveit_fake.launch add_other_geometry:=true geometry_type:=sphere

    # 加载其它mesh模型（这里加载vacuum_gripper为例，如果加载的模型是放在xarm_description/meshes/other里面，geometry_mesh_filename参数只需要传文件名）（此处以虚拟xArm7为例）
    roslaunch uf_robot_moveit_config xarm7_moveit_fake.launch add_other_geometry:=true geometry_type:=mesh geometry_mesh_filename:=package://xarm_description/meshes/vacuum_gripper/xarm/visual/vacuum_gripper.stl geometry_mesh_tcp_xyz:='"0 0 0.126"'
    ```
  - __jnt_stat_pub_rate__: joint_state_publisher的发布频率，默认为10
  - __kinematics_suffix__: 指定关节Kinematics参数文件后缀(适用于2023年8月之后出产的xArm/UF850系列)
    - 参数文件的生成: 
      ```bash
      cd src/xarm_ros/xarm_description/config/kinematics
      python gen_kinematics_params.py {robot_ip} {kinematics_suffix}

      # 注意
      # 1. robot_ip表示机械臂IP，需要连接机械臂获取实际的参数
      # 2. kinematics_suffix表示生成的参数文件的后缀，如果成功，会在xarm_description/config/kinematics/user目录下生成配置文件, 假如 kinematics_suffix 为 AAA, 那么对应的文件名如下
      #   xarm5: xarm_description/config/kinematics/user/xarm5_kinematics_AAA.yaml
      #   xarm6: xarm_description/config/kinematics/user/xarm6_kinematics_AAA.yaml
      #   xarm7: xarm_description/config/kinematics/user/xarm7_kinematics_AAA.yaml
      #   lite6: xarm_description/config/kinematics/user/lite6_kinematics_AAA.yaml
      #   uf850: xarm_description/config/kinematics/user/uf850_kinematics_AAA.yaml
      ```
    - 参数文件的使用: 在启动launch文件时指定该参数
      - 注意指定该参数之前要保证对应的配置文件存在，如果不存在，需要先通过脚本连接机械臂生成

- ### 专用参数
  - __hw_ns__: 命名空间，xarm系列默认为 __xarm__, 其它默认为 __ufactory__, 仅在gazebo/realmove启动脚本有效, 对应的服务名就是 <`hw_ns`>/<service_name>
  - __velocity_control__: 是否使用速度控制，仅在gazebo/realmove启动脚本有效, 默认为false
  - __report_type__: 上报类型，支持normal/dev/rich，默认为normal，仅在realmove启动脚本有效。 它决定了状态上报的内容和频率。具体请参考[开发者手册](https://www.cn.ufactory.cc/_files/ugd/896670_8c25a14281ce4b63814c1730c3fd82c4.pdf)中的**2.1.6. 自动上报数据格式**章节查看三种上报类型(`normal/rich/dev`)的内容区别, 默认使用的类型为 "normal"。
    - 对于需要高频率状态反馈的用户, 在启动时可以指定`report_type:=dev`, 这样`/<hw_ns>/xarm_states`和`/<hw_ns>/joint_states`话题会以**100Hz**频率更新。
    - 对于需要使用`/<hw_ns>/controller_gpio_states`话题实时查看控制器GPIO状态的用户, 请指定`report_type:=rich`, 可以在开发者手册中看到这个类型反馈的信息是最全的。
    - 不同上报类型的更新频率:   

      |   type   |    port No.   | Frequency |  GPIO topic  | F/T sensor topic | 
      |:--------:|:-------------:|:---------:|:------------:|:----------------:|
      |   normal |     30001     |    5Hz    |     不可用    |      不可用        |
      |   rich   |     30002     |    5Hz    |      可用     |       可用        | 
      |   dev    |     30003     |    100Hz  |     不可用    |        可用        |

      注: **GPIO topic** => `<hw_ns>/controller_gpio_states`. **F/T sensor topic** =>  `<hw_ns>/uf_ftsensor_ext_states` and `<hw_ns>/uf_ftsensor_raw_states`。
