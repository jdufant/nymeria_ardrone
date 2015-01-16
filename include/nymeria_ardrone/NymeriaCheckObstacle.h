#ifndef NYMERIA_CHECK_OBSTACLE_H
#define NYMERIA_CHECK_OBSTACLE_H

#include "ros/ros.h"
#include <ardrone_autonomy/Navdata.h>

void stateDroneCallback(const ardrone_autonomy::Navdata& data);

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

		//double sumError; 				//sum error for integrator
		double error; 					//error  
		//double lastEstimatedCmd;		//	
		//double lastEstimatedAngle;	//for the old "rebouclage", not useful anymore
		//double lastAngleEstimated2;	//for the old "rebouclage", not useful anymore
		//double angleEstimated;		//for rebouclage, to get the estimated command
		//double factor;				//speed factor after regulation

		ros::Subscriber sub_navdata;

		int securityDist;

		void regulation (double angleEstimated, double userCmd);
		double pilotage (const double& distToObstacle, const double& securityDist, const double& userCmd);
		double PID (const double lastError, const double estimatedCmd);
		double rebouclage(const double& angleEstimated);
		double saturationPente(const double lastCmd, const double param_saturation, double& currentCmd);
		void saturationCommande(double& cmd);
		// TODO exc
};

#endif
