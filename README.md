[nymeria_ardrone](https://sites.google.com/site/projetsecinsa/projets-2014-2015/projet-nymeria) : Obstacle detection and avoidance for ARDrone 2.0
===============

nymeria_ardrone is a [ROS](http://ros.org/ "Robot Operating System") package for [Parrot AR-Drone](http://ardrone2.parrot.com/) quadrocopter. It acts as a layer and filters drone commands sent from an external controller. It helps the drone determine if movement orders are safe or not depending on the trajectory of an obstacle and, if so, to move accordingly. In practice it contains three main modules. The first one, linked to sensors, allows to detect obstacles. The second one gets drone's command and the last one to make the link between them. With Nymeria, drones slow down and stop in front of a obstacle. They also stay within range with any obstacle allowing them to achieve missions successfully. The driver supports AR-Drone 2.0.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [How to Run](#how-to-run)

## Requirements

- *ROS*: [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- *ardrone_autonomy*: [Driver for Ardrone 1.0 & 2.0](https://github.com/AutonomyLab/ardrone_autonomy)
- *Sensor*: everyting retrieving range between drone and obstacles

## Installation

The first step is to install ROS following the [(Robot Operating System installation tutorial)](http://wiki.ros.org/ROS/Installation). We have successfully tested two versions : hydro and indigo.

Then create a [ROS workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace) according to the continuation of the previous tutorial.

In order to communicate with the drone you will need to download [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) which provide the ardrone_driver. Follow the instruction in the [installation section](https://github.com/AutonomyLab/ardrone_autonomy#installation).

When you come to this point you can download the nymeria_ardrone package using the following command in a terminal.
```bash
$ cd ~/catkin_ws/src % navigate to your catkin_workspace sources repository
$ git clone https://github.com/jdufant/nymeria_ardrone % clone (or download and unpack) the nymeria_ardrone package
$ cd ~/catkin_ws % go back to your root workspace repository
$ catkin_make % use 'catkin_make' to compile
```

## How to Run

Firstly switch on Wifi on your computer and connect it to your Ardrone© 2.0.

Such as all executable nodes with ROS, you must launch the master node :
```bash
$ cd ~/catkin_ws % navigate to your catkin_workspace
$ roscore
```

Then launch the ardrone_autonomy driver's executable node. You can either use `rosrun ardrone_autonomy ardrone_driver` or put it in a custom launch file with your desired parameters.
```bash
$ rosrun ardrone_autonomy ardrone_driver
```

Navigate to ~/catkin_ws/src/nymeria_ardrone/src/SensorInterface.cpp and find the line
*nco.inputCurFrontDist(cutValue);* Replace the 'cutValue' variable by the current distance of the front sensor of your drone. Once done, run the sensor_interface node :
```bash
$ rosrun nymeria_ardrone nymeria_sensor_interface
```

By default the security distance is 100 cm.

Launch the nymeria_command executable node using :
```bash
$ rosrun nymeria_ardrone nymeria_command
```
This node is the interface between you as a user who wish to send orders and the drone. Command are sent from keystroke detailled below.
- *ENTER* : LAND / TAKE OFF
- *Z* : move forward
- *S* : move backward
- *Q* : rotate left
- *D* : rotate right
- *SPACE* : stop

By default the security distance is 100 cm.

The last step is to run the Controller node
```bash
$ rosrun nymeria_ardrone controller
```

You are ready to go. Just stroke the appropriate key from the nymeria_command interface. Your drone will naturally keep the input security distance between any front obstacle and itself.
