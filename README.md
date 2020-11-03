# Ros package for lebot.

## Structure 1.0

#### Packages

​	1) realsense_ros
​	2) ros_lebot
​	3) cv_bridge

#### Topics

​	From realsense_ros:
​		._ /depth
​		._ /image

​	From ros_lebot:
​		._ /wheel_values [x]
​		._ /ball_distance [x]
​		._ /ball_location [x]
​		._ /basket_location [x]
​		._ /basket_distance []
​		._ /depth_and_ball_location []

#### Nodes

​	From ros_lebot:
​		._ /find_basket [x]
​		._ /vruum [x]
​		._ /depth_ball [x]
​		._ /find_ball [x]







