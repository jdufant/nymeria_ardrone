#include <nymeria_ardrone/Controller.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/Nymeria.h>

ros::NodeHandle * Controller::getNH(){
	return &nh;
}

Controller::Controller(){}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	
	Controller si;

	si.loop(si.getNH());

	return 0;
}

void Controller::loop(ros::NodeHandle * n){
	int rate = 50;
	ros::Rate loop_rate(rate);

	Nymeria nym (n, 80);
	
	while(ros::ok()){

		// hand over commands to Nymeria
		nym.validateStates();

	 	loop_rate.sleep();
	}
	return;
}