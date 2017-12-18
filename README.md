IMT-350 Robotics, II - 2017 MY BOT
========================
by, Israel Raul Tiñini Alvarez and Ruben Coarecona

Instructor: Ing. Pinaya Gutierrez Benjamin

## MAPPING AND NAVIGATION: USING THE ROS NAVIGATION STACK
The objective of this project is to implement a navigation system based on ROS for the control of a terrestrial robot that performs autonomous localization and navigation tasks.

A simulated differential vehicle model was created that satisfies the necessary requirements: the transformation tree, the odometry, the laser scanner and the base controller, whose code can be found in the folder src/mybot_description/mybot.xacro
![robot](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/robot.png)

Gmapping, mapserver and teleoperation were used to map the simulated environment in Gazebo, for this three steps were followed: create the map, save the map and load the map. The maps are in src/mybot_navigation/maps and the worlds of Gazebo are in src/mybot_gazebo/world
![mapping](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/mapeoimagen.png)

In the simulation of the autonomous navigation of the robot, ACML was used for the location and the base move package for navigation, in addition to the simulated model of the robot that was charged in Gazebo to perform the simulation, the parameters of the navigation node were configured find in src/mybot_navigation/config
![navigation](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/Evacion%20de%20Obstaculos.png)

A differential vehicle was built, which was composed mainly of two engines, an arduino and a raspberry pi 3B, and autonomous navigation was carried out with this. Autonomous navigation of a simulated and physical land vehicle could be performed, however the evasion of obstacles I just managed to make it through the simulation. For the detection of physical osbtaculos Visp_auto_track was used, because this uses the node of usb_cam which is compatible with the Raspberry
### Physical robot
![physicsRobot](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/robotFisico.png)
### Diagram of nodes and topics
![allnodes](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/todomasarduino.png)
### Serial communication
![SerialCom](https://github.com/IsraelAlvarez/MyBot/blob/master/Images/ComunicacionSerial.png)


## MAPPING 
There are basically three steps: 
* Creating the map
* Saving the map
* Loading the map.

### CREATING THE MAP
Run the following commands below. Use gmapping to map the environment, acml for localitation, the teleop to move the robot around to create an accurate and thorough map:
* Launch the Gazebo world
`roslaunch mybot_gazebo mybot_world.launch`
* Start map building
`roslaunch mybot_navigation gmapping_demo.launch`
* Launch RVIZ
`roslaunch mybot_description mybot_rviz_gmapping.launch`
* Start teleop
`roslaunch mybot_navigation mybot_teleop.launch`

[MAPPING VIDEO](https://www.youtube.com/watch?v=dSypf14WUvg&t=115s)

### SAVING THE MAP
Run the following commands below. Use MapServer to save the map. 
* Save the map using MapServer
`rosrun map_server map_saver -f ~/mybot_ws/src/mybot_navigation/maps/test_map` 
 
### LOADING THE MAP
Run the following commands below. Verify that the mapping was done well
* Launch the World
`roslaunch mybot_gazebo mybot_world.launch`
* Load the map
`roslaunch mybot_navigation amcl_demo.launch` 

## SIMULATE AUTONOMOUS NAVIGATION 
Run the following commands below. Use rviz to set navigation waypoints and the robot should move autonomously
* Launch the World
`roslaunch mybot_gazebo mybot_world.launch`
* Load the map
`roslaunch mybot_navigation amcl_demo.launch`
* Launch RVIZ 
`roslaunch mybot_description mybot_rviz_amcl.launch`

[SIMULATE AUTONOMOUS NAVIGATION  VIDEO](https://www.youtube.com/watch?v=kb35P2WhcE8&t=183s)

## SIMULATE TELEOPERATION ON THE PHYSICAL ROBOT
Run the following commands below.
* Start serial communication (Raspberry)
`rosrun rosserial_python serial_node.py /dev/ttyACM0`  
* Launch the World
`roslaunch mybot_gazebo mybot_world.launch` 
* Launch the teleoperation
`roslaunch mybot_navigation mybot_teleop.launch`
* Start the node that transforms speed messages
`rosrun odometria twist_to_motor` 
* Start the node that transforms the count of the encoders into a odometry message
`rosrun odometria diff_tf`
* Listen to the speed in PWM of the engine 1 
`rostopic echo /motor1` 
* Listen to the speed in PWM of the engine 2
`rostopic echo /motor2`  

## AUTONOMOUS NAVIGATION ON THE PHYSICAL ROBOT AND SIMULATE TELEOPERATION
Run the following commands below.
* Start serial communication (Raspberry)
`rosrun rosserial_python serial_node.py /dev/ttyACM0`  
* Start the node that transforms the count of the encoders into a odometry message
`rosrun odometria diff_tf`
* Start the node that transforms speed messages
`rosrun odometria twist_to_motor` 
* Launch the Move Base node with ACML
`roslaunch mybot_navigation amcl_demo.launch` 
* Launch RVIZ
`roslaunch mybot_description mybot_rviz_amcl.launch`
* Listen to the speed in PWM of the engine 1 
`rostopic echo /motor1` 
* Listen to the speed in PWM of the engine 2
`rostopic echo /motor2`  

[AUTONOMOUS NAVIGATION ON THE PHYSICAL ROBOT VIDEO](https://www.youtube.com/watch?v=W2Dxy_sPX_0)
