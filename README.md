# Ros package for lebot.

## Structure 1.0

#### Packages

​	1) realsense-ros
​	2) ros_lebot
​	3) cv_bridge

#### Topics

From realsense-ros:

- [x] /camera/aligned_depth_to_color/image_raw
- [x] /camera/color/image_raw

From ros_lebot:

- [x]  /move_somewhere
- [x] /ball
- [ ]  /basket

#### Nodes

From ros_lebot:

- [x]  /vruum
- [ ]  /basket_calc
- [x]  /ball_calc

#### Notes.

* If we use /camera/extrinsics/depth_to_color, then maybe it's faster? But problems would arise with the balls. Apply only with basket mask?
* No game logic applied yet, just sensor.
* Need external calculation for game movement (Do calculation, then publish to wheel_values).
* No CMAKE yet.