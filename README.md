# Context-aware and cost-effective navigation for approaching restaurant tables with lidar and ultrasound range sensors

This package presents a new system, where the LIDAR is used for SLAM, avoiding obstacles and navigating the areas between tables, using the ROS navigation stack, while cheap URF sensors and a simple, but the reliable module is responsible for navigating right up to a table using smooth movements. Using this context switch also allows the robot to avoid customers with a wide margin, but still approach the tables very closely. 

Dataset: URL link


## Docker image
To test software we have create a docker image, steps to test it:
```
$ docker pull claudiaalvarezaparicio/context-aware-table-navigation-2022:latest
$ docker run -it --rm -p 6080:80 claudiaalvarezaparicio/context-aware-table-navigation-2022:latest
```

In the web browser: http://127.0.0.1:6080/

Some launch files have been created in the navigation\_modes package in order to test the simulated environment of Budapest and Leon with the Biscee robot and both locations 3 and 15 cm.

- To launch gazebo, Rviz and the SW:

Open a terminal:
```
$ roslaunch navigation_modes <launch_file>
```
Available <launch_file>:
```
+ custom_navigation_budapest_15cm.launch
+ custom_navigation_budapest_3cm.launch
+ custom_navigation_leon_15cm.launch
+ custom_navigation_leon_3cm.launch
+ move_base_navigation_budapest_15cm.launch
+ move_base_navigation_budapest_3cm.launch
+ move_base_navigation_leon_15cm.launch
+ move_base_navigation_leon_3cm.launch
```

- To start and stop the experiment:
```
$ cd /root/
$ ./start.sh
$ ./stop.sh
```

## Results custom vs move_base navigation
TODO: BINDER link
