#ifndef NYMERIA_POLLING_H
#define NYMERIA_POLLING_H

#include "ros/ros.h"
#include <nymeria_ardrone/Nymeria.h>

class Polling
{
	public:
		Polling();

	private:
		ros::NodeHandle nh;
};

#endif