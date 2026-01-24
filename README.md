
# ROS2 Click Control (turtlebot)

Control a robot by clicking in a separate window.

Left click on upper half of the window – move forward  
Left click on lower half of the window – move backward  
Right click – stop  
This demo uses gazeboo turtlebot.

## How to run

Open a terminal in the project folder and build it:

```bash
cd ~/ROS_project
colcon build
source install/setup.bash
```
## Run full program with one command
```
ros2 launch click_control demo_turtlesim.launch.py
```
It starts both the turtlesim and control window

# Manual run

## Start the robot in terminal 1
```
ros2 run turtlesim turtlesim_node
```
## Start the control program in second terminal:
```
cd ~/ROS_project
source install/setup.bash
ros2 run click_control control_window --ros-args -r /cmd_vel:=/turtle1/cmd_vel
```
