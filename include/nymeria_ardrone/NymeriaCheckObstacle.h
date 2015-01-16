#ifndef NYMERIA_CHECK_OBSTACLE_H
#define NYMERIA_CHECK_OBSTACLE_H

#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>

void stateDroneCallback(const ardrone_autonomy::Navdata& data);

class NymeriaCheckObstacle
{
	public:

		NymeriaCheckObstacle();
		NymeriaCheckObstacle(ros::NodeHandle * n);

		void inputCurFrontDist(int cfd);

		double getSecurityDist();
		void setSecurityDist(double secDist);
		double getSensorMaxRange();
		void setSensorMaxRange(double range);

	private:
		ros::NodeHandle * nh;

		double sumError;
		double error;
		double lastCmdEstimated;
		double lastAngleEstimated;
		double lastAngleEstimated2;
		double angleEstimated;
		double cmd;
		double sensorMaxRange;

		ros::Subscriber sub_navdata;

		double pilotage ();
		void regulation (double angleEstimated);
		double PID (double lastError);
		double rebouclage(double angleEstimated);
		double saturationPente(double lastCmd);
		void saturationCommande(double& cmd);
		// TODO exc
};

#endif
