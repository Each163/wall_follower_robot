# wall_follower_robot
This ROS2 (foxy) package includes a car that can follow the wall on its right side in a gazebo simulation world. After cloning the package to your workspace/src, you can move to the root of your workspace and use the following code to build it:
```
colcon build --packages-select wall_follower_robot
``` 
To see the performance in gazebo, you can run the following codes in one terminal:
```
. install/setup.bash
ros2 launch wall_follower_robot gazebo_world.launch.py
```
Then, you can open another terminal, and run the following codes:
```
. install/setup.bash
ros2 launch wall_follower_robot controller_estimator.launch.py
```
Simulation scene:
<p align="center">
	<img height="300" src="/images/simulation_in_gazebo.gif" />
</p>

## Reference
The models of the car and the world are from automaticaddison[^1]. This simulation is also learned from this website. 

[^1]:https://automaticaddison.com
