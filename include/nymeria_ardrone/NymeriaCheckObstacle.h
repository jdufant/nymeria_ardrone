#ifndef NYMERIA_CHECK_OBSTACLE_H
#define NYMERIA_CHECK_OBSTACLE_H

#include "ros/ros.h"

class NymeriaCheckObstacle
{
	public:

		NymeriaCheckObstacle();
		NymeriaCheckObstacle(ros::NodeHandle * n, int securityDist);

		void inputCurFrontDist(int cfd);

		int getSecurityDist();
		void setSecurityDist(int sd);

	private:
		ros::NodeHandle * nh;

		int securityDist;
};

#endif