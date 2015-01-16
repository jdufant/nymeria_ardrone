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

/**
 * Central functionality of the controller: trigger Nymeria in order to actiate
 * obstacle detection and avoidance.
 **/
void Controller::loop(ros::NodeHandle * n){
	// TODO augmente it to argument
	int rate = 50;
	int errorCode;
	int lastErrorCode = 0;
	/* Modifiable loop rate */
	ros::Rate loop_rate(rate);

	Nymeria nym (n);
	
	while(ros::ok()){

		/* Trigger Nymeria */
		errorCode = nym.validateStates();
		/* Interpret return value. */
		switch (errorCode) {
			case NymeriaConstants::INIT :
				if(errorCode != lastErrorCode) printf("init\n");
				break;
			case NymeriaConstants::O_FRONT :
				if(errorCode != lastErrorCode) printf("obstacle\n");
				break;
			default :
				if(errorCode != lastErrorCode) printf("code : %d\n", errorCode);
				break;
		}

		lastErrorCode = errorCode;

	 	loop_rate.sleep();
	}
	return;
}
