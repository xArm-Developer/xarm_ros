# request: command specification for velocity executions.
# Units:
#	joint space/angles: radian/s
#	Cartesian space: mm/s, radian/s.

# speeds: the velocity list of the joints/tcp
#   For velo_move_joint_timed topic: [joint1_velocity, ..., joint7_velocity]
#   For velo_move_line_timed topic: [x_velocity, y_velocity, z_velocity, rx_velocity, ry_velocity, rz_velocity (axis-angle)]
float32[] speeds

# is_sync: this is special for velo_move_joint_timed topic, meaning whether all joints accelerate and decelerate synchronously, true for yes, false for no.
# avaiable for topic velo_move_joint_timed
bool is_sync

# is_tool_coord: this is special for velo_move_line_timed topic, meaning whether motion is in tool coordinate(true) or not(false)
# avaiable for topic velo_move_line_timed
bool is_tool_coord

# the maximum duration of the speed, over this time will automatically set the speed to 0
#   duration > 0: seconds, indicates the maximum number of seconds that this speed can be maintained
#   duration == 0: always effective, will not stop automativally
#   duration < 0: only used to be compatible with the old protocol, equivalent to 0
# avaiable for firmware_version >= 1.8.0
float32 duration