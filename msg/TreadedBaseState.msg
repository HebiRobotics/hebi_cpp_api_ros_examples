string message
# State of the robot
# 0: STARTUP
# 1: HOMING
# 2: ALIGNING
# 3: TELEOP
# 4: EMERGENCY_STOP
# 5: EXIT
int8 state
bool flipper_trajectory_active
bool base_trajectory_active
bool stable_mode
bool mstop_pressed