# ROS_project


# ROS2 Click Control (turtlesim)

Control a robot by clicking in a separate window.

Left click on upper half of the window – move forward  
Left click on lower half of the window – move backward  
Right click – stop  

This demo uses turtlesim.

## How to run

Open a terminal in the project folder and run:

```bash
cd ~/ROS_project
colcon build
source install/setup.bash
```

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
