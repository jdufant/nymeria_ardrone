#ifndef NYMERIA_CONTROLLER_H
#define NYMERIA_CONTROLLER_H

#include "ros/ros.h"
#include <nymeria_ardrone/NymeriaCheckObstacle.h>

class Controller
{
public:
		Controller();
		void loop(ros::NodeHandle * n);  
		ros::NodeHandle * getNH();

	private:
		ros::NodeHandle nh;
};

#endif