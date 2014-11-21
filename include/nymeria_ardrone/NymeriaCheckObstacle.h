#ifndef NYMERIA_CHECK_OBSTACLE_H
#define NYMERIA_CHECK_OBSTACLE_H

#include "ros/ros.h"

class NymeriaCheckObstacle
{
	public:

		NymeriaCheckObstacle();
		NymeriaCheckObstacle(ros::NodeHandle * n, double securityDist);

		void inputCurFrontDist(double cfd);

		double getSecurityDist();
		void setSecurityDist(double sd);

	private:
		ros::NodeHandle * nh;

		double securityDist;
};

#endif