# xarm_api services description

## Services
&ensp;&ensp;The services of xarm_api is the interface encapsulation of [C++ SDK](https://github.com/xArm-Developer/xArm-CPLUS-SDK), which may be a single interface call or multiple interface calls.
&ensp;&ensp;Note: The service path prefix (__xarm/__) used by rosservice is related to the startup parameters. The default *xarm* series is __xarm/__, and the *Lite* series is __ufactory/__. The following example uses xarm/, the actual use is modified according to the actual situation.

[C++ SDK DOCS](https://github.com/xArm-Developer/xArm-CPLUS-SDK/blob/master/doc/xarm_cplus_api.md)

- ### *Init*
  - ##### motion_ctrl
    - SDK API
      - `motion_enable`
    - rosservice
      ```bash
      # id: 1~7 means joint1~joint7, 8 means all joints
      # enable: 1 means enable, 0 means disable
      rosservice call /xarm/motion_ctrl ${enable} ${id}
      ```
  - ##### set_mode
    - SDK API:
      - `set_mode`
    - rosservice:
      ```bash
      # mode: 
      #   0: position mode
      #   1: servo motion mode
      #   2: joint teaching mode
      #   4: joint velocity mode
      #   5: cartesian velocity mode
      #   6: joint online trajectory planning mode
      #   7: cartesian online trajectory planning mode
      rosservice call /xarm/set_mode ${mode}
      ```
  - ##### set_state
    - SDK API:
      - `set_state`
    - rosservice:
      ```bash
      # state:
      #   0: motion state
      #   3: pause state
      #   4: stop state
      rosservice call /xarm/set_state ${state}
      ```
  - ##### clear_err
    - SDK API:
      - (if gripper added) `clean_gripper_error`
      - `clean_error`
      - `clean_warn`
      - `motion_enable`
    - rosservice:
      ```bash
      rosservice call /xarm/clear_err
      ```
  - ##### moveit_clear_err
    - SDK API:
      - (if gripper added) `clean_gripper_error`
      - `clean_error`
      - `clean_warn`
      - `motion_enable`
      - `set_mode`
      - `set_state`
    - rosservice:
      ```bash
      rosservice call /xarm/moveit_clear_err
      ```

- ### *Motion*
  - ##### go_home
    - SDK API:
      - `move_gohome`
    - rosservice:
      ```bash
      # mvvelo: speed (rad/s)
      # mvacc: acceleration (rad/s^2)
      rosservice call /xarm/go_home [] ${mvvelo} ${mvacc} 0 0
      ```
  - ##### move_joint
    - SDK API:
      - `set_servo_angle`
    - rosservice:
      ```bash
      # angles: [joint1-rad, ..., joint${dof}-rad]
      # mvvelo: speed (rad/s)
      # mvacc: acceleration (rad/s^2)
      rosservice call /xarm/move_joint ${angles} ${mvvelo} ${mvacc} 0 0
      ```
  - ##### move_jointb
    - SDK API:
      - `set_servo_angle`
    - rosservice:
      ```bash
      # angles: [joint1-rad, ..., joint${dof}-rad]
      # mvvelo: speed (rad/s)
      # mvacc: acceleration (rad/s^2)
      # radius: the blending radius between 2 straight path trajectories, 0 for no blend.
      rosservice call /xarm/move_jointb ${angles} ${mvvelo} ${mvacc} 0 ${radius}
      ```
  - ##### move_line
    - SDK API:
      - `set_position`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      rosservice call /xarm/move_line ${pose} ${mvvelo} ${mvacc} 0 0
      ```
  - ##### move_lineb
    - SDK API:
      - `set_position`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      # radius: the blending radius between 2 straight path trajectories, 0 for no blend.
      rosservice call /xarm/move_lineb ${pose} ${mvvelo} ${mvacc} 0 ${radius}
      ```
  - ##### move_line_tool
    - SDK API:
      - `set_tool_position`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      rosservice call /xarm/move_line_tool ${pose} ${mvvelo} ${mvacc} 0 0
      ```
  - ##### move_servoj
    - SDK API:
      - `set_servo_angle_j`
    - rosservice:
      ```bash
      # angles: [joint1-rad, ..., joint${dof}-rad]
      # mvvelo: speed (rad/s)
      # mvacc: acceleration (rad/s^2)
      rosservice call /xarm/move_servoj ${angles} ${mvvelo} ${mvacc} 0 0
      ```
  - ##### move_servo_cart
    - SDK API:
      - `set_servo_cartesian`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      # coord: motion coordinate system indicator, base (0) or tool(1) coordinate
      rosservice call /xarm/move_servo_cart ${pose} ${mvvelo} ${mvacc} ${coord} 0
      ```
  - ##### move_line_aa
    - SDK API:
      - `set_position_aa`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      # coord: motion coordinate system indicator, base (0) or tool(1) coordinate
      # relative: indicator of given target is relative (1) or not (0, absolute)
      rosservice call /xarm/move_line_aa ${pose} ${mvvelo} ${mvacc} 0 ${coord} ${relative}
      ```
  - ##### move_servo_cart_aa
    - SDK API:
      - `set_servo_cartesian_aa`
    - rosservice:
      ```bash
      # pose: [x(mm), y(mm), z(mm), roll(rad), pitch(rad), yaw(rad)]
      # mvvelo: speed (mm/s)
      # mvacc: acceleration (mm/s^2)
      # coord: motion coordinate system indicator, base (0) or tool(1) coordinate
      # relative: indicator of given target is relative (1) or not (0, absolute)
      rosservice call /xarm/move_servo_cart_aa ${pose} ${mvvelo} ${mvacc} 0 ${coord} ${relative}
      ```
  - ##### velo_move_joint
    - SDK API:
      - `vc_set_joint_velocity`
    - rosservice:
      ```bash
      # velocities: [joint1-velo(rad/s), ..., joint${dof}-velo(rad/s)]
      # jnt_sync: whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no
      rosservice call /xarm/velo_move_joint ${velocities} ${jnt_sync} 0
      ```
  - ##### velo_move_line
    - SDK API:
      - `vc_set_cartesian_velocity`
    - rosservice:
      ```bash
      # velocities: [velo-x(mm/s), velo-y(mm/s), velo-z(mm/s), velo-roll(rad/s), velo-pitch(rad/s), velo-yaw(rad/s)]
      # coord: whether motion is in tool coordinate(1) or not(0)
      rosservice call /xarm/velo_move_joint ${velocities} 0 ${coord}
      ```
  - ##### velo_move_joint_timed
    - SDK API:
      - `vc_set_joint_velocity`
    - rosservice:
      ```bash
      # velocities: [joint1-velo(rad/s), ..., joint${dof}-velo(rad/s)]
      # is_sync: whether all joints accelerate and decelerate synchronously, 1 for yes, 0 for no
      # duration: the maximum duration of the speed, over this time will automatically set the speed to 0
      rosservice call /xarm/velo_move_joint_timed ${velocities} ${is_sync} 0 ${duration}
      ```
  - ##### velo_move_line_timed
    - SDK API:
      - `vc_set_cartesian_velocity`
    - rosservice:
      ```bash
      # velocities: [velo-x(mm/s), velo-y(mm/s), velo-z(mm/s), velo-roll(rad/s), velo-pitch(rad/s), velo-yaw(rad/s)]
      # is_tool_coord: whether motion is in tool coordinate(1) or not(0)
      # duration: the maximum duration of the speed, over this time will automatically set the speed to 0
      rosservice call /xarm/velo_move_line_timed ${velocities} 0 ${is_tool_coord} {duration}
      ```

- ### *Tool GPIO* (ionum from 1 to 2)
  - ##### set_digital_out
    - SDK API:
      - `set_tgpio_digital`
    - rosservice:
      ```bash
      # io_num: io num, 1: DO0, 2: DO1
      # value: 0/1
      rosservice call /xarm/set_digital_out ${io_num} ${value}
      ```
  - ##### get_digital_in
    - SDK API:
      - `get_tgpio_digital`
    - rosservice:
      ```bash
      rosservice call /xarm/get_digital_in
      ```
  - ##### get_analog_in
    - SDK API:
      - `get_tgpio_analog`
    - rosservice:
      ```bash
      # io_num: io num, 1: AI0, 2: AI1
      rosservice call /xarm/get_analog_in ${io_num}
      ```
- ### *Controller GPIO* (ionum from 1 to 16)
  - ##### set_controller_dout
    - SDK API:
      - `set_cgpio_digital`
    - rosservice:
      ```bash
      # io_num: io num, 1 ~ 16, (1: CO0, 9: DO0)
      # value: 0/1
      rosservice call /xarm/set_controller_dout ${io_num} ${value}
      ```
  - ##### get_controller_din
    - SDK API:
      - `get_cgpio_digital`
    - rosservice:
      ```bash
      # io_num: io num, 1 ~ 16, (1: CI0, 9: DI0)
      rosservice call /xarm/get_controller_din ${io_num}
      ```
  - ##### set_controller_aout
    - SDK API:
      - `set_cgpio_analog`
    - rosservice:
      ```bash
      # io_num: io num, 1: AO0, 2: AO1
      # value: value
      rosservice call /xarm/set_controller_aout ${io_num} ${value}
      ```
  - ##### get_controller_ain
    - SDK API:
      - `get_cgpio_analog`
    - rosservice:
      ```bash
      # io_num: io num, 1: AI0, 2: AI1
      rosservice call /xarm/get_controller_ain ${io_num}
      ```

- ### *XArm Gripper*
  - ##### gripper_config
    - SDK API:
      - `set_gripper_mode`
      - `set_gripper_enable`
      - `set_gripper_speed`
    - rosservice:
      ```bash
      # velocity: pulse velocity, 1 ~ 5000
      rosservice call /xarm/gripper_config ${velocity}
      ```
  - ##### gripper_move
    - SDK API:
      - `set_gripper_position`
    - rosservice:
      ```bash
      # pos: pulse pos, -100 ~ 850
      rosservice call /xarm/gripper_move ${pos}
      ```
  - ##### gripper_state
    - SDK API:
      - `get_gripper_err_code`
      - `get_gripper_position`
    - rosservice:
      ```bash
      rosservice call /xarm/gripper_state 
      ```

- ### *XArm Vacuum Gripper*
  - ##### vacuum_gripper_set
    - SDK API:
      - `set_vacuum_gripper`
    - rosservice:
      ```bash
      # on: 1: open, 0: close
      rosservice call /xarm/vacuum_gripper_set ${on}
      ```

- ### *Lite Gripper*
  - ##### open_lite6_gripper
    - SDK API:
      - `open_lite6_gripper`
    - rosservice:
      ```bash
      rosservice call /xarm/open_lite6_gripper
      ```
  - ##### close_lite6_gripper
    - SDK API:
      - `close_lite6_gripper`
    - rosservice:
      ```bash
      rosservice call /xarm/close_lite6_gripper
      ```
  - ##### stop_lite6_gripper
    - SDK API:
      - `stop_lite6_gripper`
    - rosservice:
      ```bash
      rosservice call /xarm/stop_lite6_gripper
      ```

- ### *Force torque Sensor*
  - ##### ft_sensor_enable
    - SDK API:
      - `ft_sensor_enable`
    - rosservice:
      ```bash
      # enable: 1: enable, 0: disable
      rosservice call /xarm/ft_sensor_enable ${enable}
      ```
  - ##### ft_sensor_app_set
    - SDK API:
      - `ft_sensor_app_set`
    - rosservice:
      ```bash
      # app_code
      #   0: non-force mode
      #   1: impendance control
      #   2: force control
      rosservice call /xarm/ft_sensor_app_set ${app_code}
      ```
  - ##### ft_sensor_set_zero
    - SDK API:
      - `ft_sensor_set_zero`
    - rosservice:
      ```bash
      rosservice call /xarm/ft_sensor_set_zero
      ```
  - ##### ft_sensor_cali_load
    - SDK API:
      - `ft_sensor_cali_load`
      - `save_conf`
    - rosservice:
      ```bash
      # data: iden load result
      # association_setting_tcp_load: association setting tcp load or not
      rosservice call /xarm/ft_sensor_cali_load ${data} ${association_setting_tcp_load}
      ```
  - ##### ft_sensor_iden_load
    - SDK API:
      - `ft_sensor_iden_load`
    - rosservice:
      ```bash
      rosservice call /xarm/ft_sensor_iden_load
      ```
  - ##### get_ft_sensor_error
    - SDK API:
      - `get_ft_sensor_error`
    - rosservice:
      ```bash
      rosservice call /xarm/get_ft_sensor_error
      ```

- ### *Tool Modbus*
  - ##### config_tool_modbus
    - SDK API:
      - `set_state`
      - `set_tgpio_modbus_baudrate`
      - `set_tgpio_modbus_timeout`
    - rosservice:
      ```bash
      # baudrate: baudrate
      # timeout_ms: timeout(ms)
      rosservice call /xarm/config_tool_modbus ${baudrate} ${timeout_ms}
      ```
  - ##### set_tool_modbus
    - SDK API:
      - `getset_tgpio_modbus_data`
    - rosservice:
      ```bash
      # send_data: modbus data
      # respond_len: the length of the response modbus data
      rosservice call /xarm/getset_tgpio_modbus_data ${send_data} ${respond_len}
      ```
  - ##### get_tgpio_modbus_baudrate
    - SDK API:
      - `get_tgpio_modbus_baudrate`
    - rosservice:
      ```bash
      rosservice call /xarm/get_tgpio_modbus_baudrate
      ```
  - ##### set_tgpio_modbus_timeout
    - SDK API:
      - `set_tgpio_modbus_timeout`
    - rosservice:
      ```bash
      # timeout_ms: timeout(ms)
      # is_transparent_transmission: is transparent transmission or not
      rosservice call /xarm/set_tgpio_modbus_timeout ${timeout_ms} ${is_transparent_transmission}
      ```
  - ##### getset_tgpio_modbus_data
    - SDK API:
      - `getset_tgpio_modbus_data`
    - rosservice:
      ```bash
      # send_data: modbus data
      # respond_len: the length of the response modbus data
      # host_id: host id, 9: END RS485, 10: Controller RS485
      # is_transparent_transmission: is transparent transmission or not
      # use_503_port: whether to use port 503 for communication
      rosservice call /xarm/getset_tgpio_modbus_data ${send_data} ${respond_len} ${host_id} ${is_transparent_transmission} ${use_503_port}
      ```

- ### *Get*
  - ##### get_servo_angle
    - SDK API:
      - `get_servo_angle`
    - rosservice:
      ```bash
      rosservice call /xarm/get_servo_angle
      ```
  - ##### get_position_rpy
    - SDK API:
      - `get_position`
    - rosservice:
      ```bash
      rosservice call /xarm/get_position_rpy
      ```
  - ##### get_position_axis_angle
    - SDK API:
      - `get_position_aa`
    - rosservice:
      ```bash
      rosservice call /xarm/get_position_axis_angle
      ```

- ### *Set*
  - ##### set_tcp_offset
    - SDK API:
      - `set_tcp_offset`
      - `save_conf`
    - rosservice
      ```bash
      # x/y/z: mm
      # roll/pitch/yaw: rad
      rosservice call /xarm/set_tcp_offset ${x} ${y} ${z} ${roll} ${pitch} ${yaw}
      ```
  - ##### set_load
    - SDK API:
      - `set_tcp_load`
      - `save_conf`
    - rosservice
      ```bash
      # mass: kg
      # xc: x center of mass (mm)
      # yc: y center of mass (mm)
      # zc: z center of mass (mm)
      rosservice call /xarm/set_load ${mass} ${xc} ${yc} ${zc}
      ```
  - ##### set_max_acc_joint
    - SDK API:
      - `set_joint_maxacc`
    - rosservice
      ```bash
      # maxacc: joint max acc, rad/s^2, 0 ~ 20
      rosservice call /xarm/set_max_acc_joint ${maxacc}
      ```
  - ##### set_max_acc_line
    - SDK API:
      - `set_tcp_maxacc`
    - rosservice
      ```bash
      # maxacc: tcp max acc, mm/s^2, 0 ~ 50000
      rosservice call /xarm/set_max_acc_line ${maxacc}
      ```
  - ##### set_collision_rebound
    - SDK API:
      - `set_collision_rebound`
    - rosservice
      ```bash
      # on: 1: on, 0: off
      rosservice call /xarm/set_collision_rebound ${on}
      ```
  - ##### set_collision_sensitivity
    - SDK API:
      - `set_collision_sensitivity`
    - rosservice
      ```bash
      # sens: collision sensitivity, 0 ~ 5
      rosservice call /xarm/set_collision_sensitivity ${sens}
      ```
  - ##### set_teach_sensitivity
    - SDK API:
      - `set_teach_sensitivity`
    - rosservice
      ```bash
      # sens: teach sensitivity, 1 ~ 5
      rosservice call /xarm/set_teach_sensitivity ${sens}
      ```
  - ##### set_world_offset
    - SDK API:
      - `set_world_offset`
      - `save_conf`
    - rosservice
      ```bash
      # x/y/z: mm
      # roll/pitch/yaw: rad
      rosservice call /xarm/set_world_offset ${x} ${y} ${z} ${roll} ${pitch} ${yaw}
      ```
  - ##### set_fence_mode
    - SDK API:
      - `set_fence_mode`
    - rosservice
      ```bash
      # on: 1: on, 0: off
      rosservice call /xarm/set_fence_mode ${on}
      ```
  - ##### set_reduced_mode
    - SDK API:
      - `set_reduced_mode`
    - rosservice
      ```bash
      # on: 1: on, 0: off
      rosservice call /xarm/set_reduced_mode ${on}
      ```
  - ##### set_tcp_jerk
    - SDK API:
      - `set_tcp_jerk`
    - rosservice
      ```bash
      # jerk: tcp max acc, mm/s^3
      rosservice call /xarm/set_tcp_jerk ${jerk}
      ```
  - ##### set_joint_jerk
    - SDK API:
      - `set_joint_jerk`
    - rosservice
      ```bash
      # jerk: joint jerk, rad/s^3
      rosservice call /xarm/set_joint_jerk ${jerk}
      ```

- ### *Trajectory*
  - ##### set_recording
    - SDK API:
      - `start_record_trajectory` or `stop_record_trajectory`
    - rosservice
      ```bash
      # on: 1: start, 0: stop
      rosservice call /xarm/set_recording ${on}
      ```
  - ##### save_traj
    - SDK API:
      - `save_record_trajectory`
    - rosservice
      ```bash
      # filename: the trajectory filename to save
      # timeout: save timeout (second)
      rosservice call /xarm/save_traj ${filename} ${timeout}
      ```
  - ##### play_traj
    - SDK API:
      - `playback_trajectory`
    - rosservice
      ```bash
      # filename: the trajectory filename to playback
      # times: repeat times
      # speed_factor: speed factor, 1/2/4
      rosservice call /xarm/play_traj ${filename} ${times} ${speed_factor}
      ```
