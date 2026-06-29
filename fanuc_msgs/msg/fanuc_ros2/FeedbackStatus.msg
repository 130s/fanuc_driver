#
# Msg to provide robot status periodically
#

std_msgs/Header header

# In error state or not.
bool in_error

# The cause of error when in_error is true.
int32 error_code

# Manual mode or AUTO mode.
# Manual mode : TP is enabled.
# AUTO mode : TP is disabled.
# warning : this doesn't indicate AUTO or T1T2 mode
int32 mode

int32 MANUAL=0
int32 AUTO=1

# Motion command from ROS is running or not.
bool in_motion

# The newtwork connection status between ROS node and the robot controller
bool connected

# Servo is ON.
bool drives_powered

# Trajectory is acceptable
bool motion_possible

#
int32 test1
int32 test2
