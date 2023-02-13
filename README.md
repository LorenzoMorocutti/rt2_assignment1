# Description of the branch

The branch contains: 
* *rt2_assignment1* : folder in which we can find the scripts and the launcher for the ros1 side. You have to place this folder in your ros1 workspace; 
* *rt2_assingment1_ros2* : folder in which we can find the cpp files and the launcher for the ros2 side. You have to place this folder in your ros2 workspace;
* three .sh scipts to set the environment needed

To launch the simulation, we have to follow some steps.
First of all, we have to open three different tabs in the terminal. In the first one, in the root folder, source to ros1 with the command:
```
source ros.sh
```
then launch the gazebo simulation and the .py nodes with the line:
```
roslaunch rt2_assignment1 sim.launch
```

After that go to the second tab and source to ros12, with the command:
```
source ros12.sh
```
then go to the ros2_ws/src folder and launch the ros_bridge with the command line:
```
ros2 run ros1_bridge dynamic_bridge
```

In the end, go to the third tab and source to ros2 with the command:
```
source ros2.sh
```
then go to ros2_ws/src folder and launch the ros2 simulation part with the command line:
```
ros2 launch rt2_assignment1_ros2 sim.py
```

At this point return to the first tab where the terminal will ask you to press <1> to start the robot and <0> to stop it.