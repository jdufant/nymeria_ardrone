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

		double sommeError;
		double error;
		double cmdEstimePrec;
		double angleEstimePrec;
		double angleEstimePrec2;
		double cmd;

		int securityDist;
		double pilotage ();
		void regulation (double angleEstime);
		double PID (double errorPrec);
		double rebouclage(double angleEstime);
		double saturationPente(double cmdPrec);
		// TODO exc
};

#endif