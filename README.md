# Ros package for lebot.

## Structure 1.0

#### Packages used

- realsense-ros
- ds4_driver
- rosbridge_suite
- vision_opencv

#### Launch files

For alpha demo:

```bash
cd test_lebot
source devel/setup.bash
roslaunch lebot pre.launch
```

For debugging logic:

```bash
cd test_lebot
source devel/setup.bash
roslaunch lebot debug_logic.launch
```

For debugging masks:

```bash
cd test_lebot
source devel/setup.bash
roslaunch lebot debug_mask.launch
```

For manual control:

```bash
cd test_lebot
source devel/setup.bash
roslaunch lebot manual_control.launch
```

where test_lebot is your ros environment.

#### Nodes

- /ControllerToRos

  Mapping of inputs. Publishes each input to the corresponding topic.

- /GameLogic

  Logic node. Susbcribes to ball, basket, and referee and does actions according to each state it is at.

- /iFindBalls

  Finds balls :p

- /iFIndBaskets

  Finds the basket and gets updated from referee changes to basket color.

- /iAndRef

  Subscribes to messages from the bridge of html and ROS and makes the corresponding changes.

#### Useful command when debugging. 

rostopic pub /referee lebot/Ref_Command "{'command':'resume'}"

rostopic pub /referee lebot/Ref_Command "{'command':'pause'}"

rostopic pub /color_ref std_msgs/String "{'data':'blue'}"

rostopic pub /color_ref std_msgs/String "{'data':'red'}"

#### Needs rework

- In logic node, go_action.

#### Missing

- Check if websocket from referee is working properly.
  - Change ref2.html websocket address to correct one.
- Localization or court limits.
  - A bit too advance :(



