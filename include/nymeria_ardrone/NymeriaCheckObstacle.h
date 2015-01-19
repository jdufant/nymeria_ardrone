#ifndef NYMERIA_CHECK_OBSTACLE_H
#define NYMERIA_CHECK_OBSTACLE_H

#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>

void stateDroneCallback(const ardrone_autonomy::Navdata& data);

/**
* Definition of the class NymeriaCheckObstacle, that declares all functionalities
* in order to allow for obstacle detection.
*/
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
	
		double error;
		double sensorMaxRange;

		ros::Subscriber sub_navdata;
		ros::NodeHandle * nh;

		void regulation (double angleEstimated, double userCmd);
		double pilotage (const double& distToObstacle, const double& securityDist, const double& userCmd);
		double PID (const double lastError, const double estimatedCmd);
		double rebouclage(const double& angleEstimated);
		double saturationPente(const double lastCmd, const double param_saturation, double& currentCmd);

		void saturationCommande(double& cmd);
		// TODO exc
};

#endif
