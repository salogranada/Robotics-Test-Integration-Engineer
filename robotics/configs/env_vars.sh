# -----------------------------------------------------------------------------
# MOTION CONTROL CONSTANTS

export SPEED_CONTROLLER_KP_THROTTLE=0.50    # [float] Proportional constant for linearPID control
export SPEED_CONTROLLER_KI_THROTTLE=0.23    # [float] Integral constant for linear PID control
export SPEED_CONTROLLER_KD_THROTTLE=0.0     # [float] Derivative constant for linear PID control
export SPEED_CONTROLLER_FF_THROTTLE=1.0     # [float] Feed Forward constant for linear PID control

export SPEED_CONTROLLER_KP_STEERING=0.75    # [float] Proportional constant for angular PID control
export SPEED_CONTROLLER_KI_STEERING=0.2     # [float] Integral constant for angular PID control
export SPEED_CONTROLLER_KD_STEERING=0.0     # [float] Derivative constant for angular PID control
export SPEED_CONTROLLER_FF_STEERING=1.0     # [float] Feed Forward constant for angular PID control

export SPEED_CONTROLLER_THROTTLE_CONTROL=1 # [int-bool] Enable linear speed control 
export SPEED_CONTROLLER_STEERING_CONTROL=1 # [int-bool] Enable angular speed control 

# LINEAR AND ANGULAR SPEED
export SPEED_CONTROLLER_MAX_LIN_VEL=1.37        # [float][m/s] The maximum linear speed of the robot
export SPEED_CONTROLLER_LINEAR_ACC_FACTOR=2.0   # [float][m/s2] Acceleration factor to control the linear soft speed spline
export SPEED_CONTROLLER_ANGULAR_ACC_FACTOR=1.2  # [float][m/s2] Acceleration factor to control the angular soft speed spline
export SPEED_CONTROLLER_MAX_ANG_VEL_MOVING=1.0  # [float][rad/s] The maximum angular speed of the robot when moving forward and turning at the same time
export SPEED_CONTROLLER_MAX_ANG_VEL_STATIC=1.3  # [float][rad/s] The maximum angular speed of the robot when its only turning along its axis
export SPEED_CONTROLLER_LIN_VEL_BASE=0.2        # [float][%] The speed percentage below which the angular speed of the robot will be the minimum
export SPEED_CONTROLLER_LIN_VEL_TOP=0.8         # [float][%] The speed percentage above which the angular speed of the robot will be the maximum
export SPEED_CONTROLLER_MAX_YAW_ANG=0.785398    # [float][rad] Radians to define the max allowed angle to enabled/disable the angle control

# WHEEL ODOMETRY
export ODOMETRY_CHASSIS_TRACK=0.392 # [float] Robot base tack (Important for kinematics)
export ODOMETRY_WHEEL_RADIUS=0.079  # [float] Wheel radius (Important for kinematics and dynamics)

# -----------------------------------------------------------------------------
# MOTORS RPM
export CONVERTER_WHEEL_MAX_RPM=165.0   # [float][rpms] Maximum allow RPM to our current wheels
export CONVERTER_WHEEL_SPD_FACTOR=17.28 # [float][factor] Factor for max velocity including the radius 
export CHASSIS_PUBLISH_TIME=180             # [int][milliseconds] Period to publish 0 data if there is not a velocity/position reference to avoid disarm the chassis