# origarm_uw
 for under water project

 ## configure joystick in ROS
 refer to [http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick]

 ## modifications compared with origarm_ros package (for under water proj)
 configuration of under water arm is composed of 4 segments with 4 actuators in each segment
 instead of specific pressure control, only on/off command will be given, as in "cmd_arduino"

 add "cmd_arduino" in Valve.msg, detailed computation listed in ABLController.cpp
 parameters changed & listed in myData.h


