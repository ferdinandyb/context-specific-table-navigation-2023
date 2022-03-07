# Context-aware navigation for approaching restaurant tables with LIDAR and URF sensors

[![DOI](https://zenodo.org/badge/460363860.svg)](https://zenodo.org/badge/latestdoi/460363860)

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

## Dataset
The data were saved and organized taking into account the environment where the data have been recorded (real or simulated), the robots used to collect the data (Orbi-One or Biscee), the location from, and location to the robot were moving (LB, L1, L2, L3, L4), the distance to the table of the locations (3 or 15 cm) and the navigation mode used (context-aware or contextless). In that way the variables and values that will be found in the dataset are:

**real** *(int)*            Enviroment where the data is gathered (*1 = real , 0 = simulation*)	    
**robot** *(string)*        Robot used to obtain the data (*Orbi-One, Biscee*)	    
**type** *(string)*	        Navigation mode used (*contextless, context-aware*)     
**tabledist** *(int)*       Distance to the table from defined locations (*3, 15 (cm)* )    
**going_from** *(string)*	Source location (*LB, L1, L2, L3, L4*)  
**going_to** *(string)*	    Destination location (*LB, L1, L2, L3, L4*)     
**time** *(seconds)*	    Time spent to reach the final position  
**scaled_time** *(seconds)*	Time spent to reach the final position using the context-aware data as base    
**distance** *(meters)*	    Euclidean distance to the final location after robot stops  
**orientation** *(degrees)* Orientation distance to the final location after robot stops    


## Results contextless vs context-aware navigation


[![Binder](https://mybinder.org/badge_logo.svg)](https://mybinder.org/v2/gh/ferdinandyb/context-aware-table-navigation-2022/HEAD?labpath=%2Fdataset%2Fresults.ipynb)
