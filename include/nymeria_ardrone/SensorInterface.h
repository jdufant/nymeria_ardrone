#ifndef NYMERIA_SENSOR_INTERFACE_H
#define NYMERIA_SENSOR_INTERFACE_H

#include "ros/ros.h"
#include <nymeria_ardrone/NymeriaCheckObstacle.h>

class SensorInterface
{
	public:
		SensorInterface();
		void loop(ros::NodeHandle * n);
		ros::NodeHandle * getNH();
	private:
		void cutBuffer(char* bufferIn, int size, int& valeur);
		ros::NodeHandle nh;
};

#endif
