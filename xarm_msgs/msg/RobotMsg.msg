# feedback information of the controlled robot
Header header

# state of robot:
#	1: RUNNING, executing motion command.
#	2: SLEEPING, not in execution, but ready to move.
#	3: PAUSED, paused in the middle of unfinished motion.
#	4: STOPPED, not ready for any motion commands.
#	5: CONFIG_CHANGED, system configuration or mode changed, not ready for motion commands.
int16 state

# mode of robot:
#	0 for POSITION mode.(position control by xarm controller box, execute api standard commands)
#	1 for SERVOJ mode. (Immediate execution towards received joint space target, like a step response)
#	2 for TEACHING_JOINT mode. (Gravity compensated mode, easy for teaching)
int16 mode

# cmdnum: number of commands waiting in the buffer.
int16 cmdnum

# mt_brake: if translated to binary digits, each bit represent one axis, 1 for brake enabled, 0 for brake disabled
int16 mt_brake

# mt_able: if translated to binary digits, each bit represent one axis, 1 for servo control enabled, 0 for servo disabled
int16 mt_able

# error code (if non-zero)
int16 err

# warning code (if non-zero)
int16 warn

# current joint angles expressed in radian.
float32[] angle

# current TCP Cartesian position expressed in mm(position), radian(orientation)  
float32[6] pose

# TCP offset from center of flange, with respect to tool frame.
float32[6] offset