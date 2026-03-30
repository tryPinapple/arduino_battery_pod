ASUQTR ROS Power Node
====================

A ROS node for ASUQTR measuring battery packs tensions via serial, from Arduinos to Nvidia Jetson Xavier.

This package serves the purpose reading serialized data coming from the Arduinos. Those 2 Arduinos are both connected to a single battery pack. They read and convert analogic tensions to digital before sending on the Nvidia Jetson Xavier. From there, the tension values are sent as messages on the ROS network. 



Proprietary License
----------------------

Copyright (c) 2020 ASUQTR <admin@asuqtr.com>



QuickStart Guide
----------------


This assumes that you have a Nvidia Jetson Xavier device with node running on it and 2 Arduinos running the code contained in this repo. Those 2 arduinos shall be connected to the Xavier via USB and have the specific device names : /dev/Arduino0 and /dec/Arduino1. This also assumes that you have already created a [catkin workspace][1]

Build:

1. cd ~/catkin_ws/src
2. <pre><code>git clone https://bitbucket.asuqtr.com/scm/subuqtr/power-node.git</code></pre>
3. cd ..
4. catkin_make


Run:

5. (Terminal 1) <pre><code>roscore</code></pre>
6. (Terminal 2) <pre><code>roslaunch asuqtr_power_node power_node.launch</code></pre>
7. (Terminal #) ctrl+c to quit



Overview 
--------

#### Power node

This node provides direct information about the tensions of both battery packs directly as topics on the ROS network.


#### power_node.launch

This launch file starts the node with the required ROS params if needed. If the needs for the project change, you should modify those values. 


References 
----------

[1]: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment "ROS Workspace Tutorial"




