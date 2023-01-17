# wall_follower_robot
This ROS2 (foxy) package includes a car that can follow the wall on its right side in a gazebo simulation world. After cloning the package to your workspace/src, you can move to the root of your workspace and use the following code to build it:
```
colcon build --packages-select wall\_follower\_robot
``` 
To see the performance in gazebo, you can run the following codes in one terminal:
```
. install/setup.bash
ros2 launch two_wheeled_robot_mine gazebo_world.launch.py
```
Then, you can open another terminal, and run the following codes:
```
. install/setup.bash
ros2 launch two_wheeled_robot_mine controller_estimator.launch.py
```
Simulation scene:
<p align="center">
	<img height="300" src="/images/simulation_in_gazebo.png" />
</p>
