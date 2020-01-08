# iiwa-variable-joint-admittance
wip

## Dependencies

- Matlab R2019b
- Simulink
- Loads of toolboxes, e.g coder, Robotics System...
- Run `rosAddons` in Command Window, install ROS Toolbox interface for ROS Custom Messages by MathWorks Robotics Team.

## Setup
- Configure ROS MASTER env. in Command Window with `setenv('ROS_MASTER_URI','http://master_ip:11311')` check with `getenv('ROS_MASTER_URI')`.
- Connect to ROS master, e.g. in simulink go to Robot pane and click Connect to Robot, enter address (without port) etc.
