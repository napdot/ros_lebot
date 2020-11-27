# Ros package for lebot.

## Structure 1.0

#### Packages

​	1) realsense-ros
​	2) ros_lebot
​	3) cv_bridge
​    	4) ds4_driver
	5) rosbridge_suite

#### Topics

From realsense-ros:

- [x] /camera/aligned_depth_to_color/image_raw
- [x] /camera/color/image_raw

From ros_lebot:

- [x]  /move_somewhere
- [x] /ball
- [x]  /basket

#### Nodes

From ros_lebot:

- [x]  /vruum
- [x]  /basket_calc
- [x]  /ball_calc

#### Additional scripts

- [ ] wheel_calc_ball_basket_orient.py ---> Publish to move_somewhere.

#### Missing.

* Game logic
* Camera parameter script
* CMAKE file.
* DS4_driver.