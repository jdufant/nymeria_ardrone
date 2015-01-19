[nymeria_ardrone](https://sites.google.com/site/projetsecinsa/projets-2014-2015/projet-nymeria) : Obstacle detection and avoidance for ARDrone 2.0
===============

nymeria_ardrone is a [ROS](http://ros.org/ "Robot Operating System") package for [Parrot AR-Drone](http://ardrone2.parrot.com/) quadrocopter. It acts as a layer and filters drone commands sent from an external controller. It helps the drone determine if movement orders are safe or not depending on the trajectory of an obstacle and, if so, to move accordingly. In practice it contains three main modules. The first, linked to sensors, allows the drone to detect an obstacle. The second gets drone commands and the last makes the link between them. User defines radius of an obstacle and drone is controlled by Nymeria to slow down and stop in front of it. The driver supports AR-Drone 2.0.

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
- [How to run it](#how-to-run)
- [How does it work](#how-does-it-work)

## Requirements

- *ROS*: [Robot Operating System](http://wiki.ros.org/ROS/Installation)
- *ardrone_autonomy*: [Driver for Ardrone 1.0 & 2.0](https://github.com/AutonomyLab/ardrone_autonomy)
- *Sensor*: any kind of tool enabling to retrieve range between drone and front obstacles

## Installation

The first step is to install ROS following the [(Robot Operating System installation tutorial)](http://wiki.ros.org/ROS/Installation). We have successfully tested two versions : hydro and indigo.

Then create a [ROS workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace).

In order to communicate with the drone you will need to download [ardrone_autonomy](https://github.com/AutonomyLab/ardrone_autonomy) which provide the ardrone_driver. Follow the instruction in the [installation section](https://github.com/AutonomyLab/ardrone_autonomy#installation).

Navigate to your catkin_workspace sources repository.
> $ cd ~/catkin_ws/src

Download the nymeria_ardrone package using the following command in a terminal.
> $ git clone https://github.com/jdufant/nymeria_ardrone

You might prefer to reach [nymeria_ardrone webpage](https://github.com/jdufant/nymeria_ardrone) and download and unpack the nymeria_ardrone package.

Go back to your root workspace repository.
> $ cd ~/catkin_ws

Use the catkin_make command to compile
> $ catkin_make

## How to run it

First switch on Wifi on your computer and connect it to your Ardrone 2.0.

You must launch the master node. Navigate to your catkin_workspace (`$ cd ~/catkin_ws`) and type the following command :
> $ roscore

Then launch the ardrone_autonomy driver's executable node. You can use :
> $ rosrun ardrone_autonomy ardrone_driver

Or put it in a custom launch file with your desired parameters.

Navigate to ~/catkin_ws/src/nymeria_ardrone/src/SensorInterface.cpp and find the line
*nco.inputCurFrontDist(cutValue);* Replace the 'cutValue' variable by the current distance of the front sensor of your drone. Once done, run the sensor_interface node :
> $ rosrun nymeria_ardrone nymeria_sensor_interface

By default the security distance is 100 cm.
To change it just call the setSecurityDist(double secDist) from the class NymeriaCheckObstacle.
~~~~~~~~~~~~~{.cpp}
double getSecurityDist();
void setSecurityDist(double secDist);
~~~~~~~~~~~~~

By default the sensor max range is 350 cm.
To change it just call the setSensorMaxRange(double range) from the class NymeriaCheckObstacle.
~~~~~~~~~~~~~{.cpp}
double getSensorMaxRange();
void setSensorMaxRange(double range);
~~~~~~~~~~~~~

Launch the nymeria_command executable node using :
> $ rosrun nymeria_ardrone nymeria_command

This node is the interface between you as a user who wish to send orders and the drone. Command are sent from keystroke detailled below.
- *ENTER* : LAND / TAKE OFF
- *Z* : move forward
- *S* : move backward
- *Q* : rotate left
- *D* : rotate right
- *UP* : move up
- *DOWN* : move down
- *i* : move down
- *k* : move down
- *o* : move down
- *l* : move down
- *p* : move down
- *m* : move down
- *SPACE* : stop

The last step consists to run the launch the Controller node
> $ rosrun nymeria_ardrone controller

You are ready to go. Just stroke the appropriate key from the nymeria_command interface. Your drone will naturally keep the inputed security distance between any front obstacle and itself.

## How does it work
