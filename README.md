ROSPlan_interface_Turtlebot2
============================

This package makes the robot to rotate around quickly so that the laser scans (white lines created in Rviz when robot is facing an obstacle) can be matched to the map in order to localise the vehicle. Instead of manually giving a pose estimate of the robot in RViz, this component will be able to "_localise_" the robot in current map; so that pose of robot in  `/base_link` will match actual location in `/map`.  

RPLocaliser.cpp file retrieves waypoint parameters from "waypoint.txt file in rosplan_demos/scripts directory. Make sure you create this .txt file including two random waypoints, such as: 

```sh
wp_name [0, 0, 0.234] 
wp_name2 [5, 4, 0.234] 
```
 
This parameter file needs to be included in `interfaced_planning_system.launch` file by adding the following command: 
 
```<param name="waypoint_file" value = "$(find rosplan_demos)/scripts/waypoints.txt" /> ```
 
Make sure that this package node is included in the same launch file: 
 
```sh
<node name="rosplan_interface_localise" pkg="rosplan_interface_turtlebot" type="rplocaliser" respawn="false" output="screen"> 
</node> 
```
 
It's desired to perform this action before visiting any waypoints. Therefore, the PDDL action below, called "_localise_", is added to the domain, and a propositional condition of _(localised ?v)_ is added to existing _goto_waypoint_ action. 
         
```sh
 (:durative-action localise 
  :parameters (?v - robot) 
  :duration ( = ?duration 10) 
  :effect (and (at end (localised ?v))))) 
```
 
When turtlebot_explore.bash file is launched, planning system will generate a valid plan (using POPF), like: 

```sh 
0.000: (localise kenny)  [10.000] 
10.001: (goto_waypoint kenny wp0 wp0)  [10.000] 
20.002: (goto_waypoint kenny wp0 wp1)  [10.000] 
30.003: (goto_waypoint kenny wp1 wp2)  [10.000] 
``` 
And, it can be observed that Planning System (PS) dispatches "_localise_" action, and is received by rosplan_interface_localise, while robot is turning around to localise itself in the simulation:   
```sh  
KCL: (PS) Dispatching action [0, localise, 10.037968, 10.000000] 
KCL: (Localiser) action received 
``` 

***

You can also observe that AMCL particle cloud gets tightened after localisation, because robot gets a better pose estimation after this action. For more information about ROS Navigation, please visit: http://wiki.ros.org/navigation

![wuegweh](http://cdn.makeagif.com/media/5-26-2015/iOlcPH.gif)

***

For more detailed information about this tutorial, please visit our [ROSPlan Wiki Page](https://github.com/KCL-Planning/ROSPlan/wiki), and follow tutorial pages on the right.
