#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>


/**
* \file NymeriaCheckObstacle.cpp
* \class NymeriaCheckObstacle NymeriaCheckObstacle.h
* \author Team-Nymeria
* \version 0.2
* \date 18th of January 2015
*/


double pitch = 0.0;
int droneState = 0;


/**
* \brief callback function for the subscriber sub_navdata
* gets the pitch of the drone and its state
* \param data variable where the value is stored, must be const
*/
void stateDroneCallback(const ardrone_autonomy::Navdata& data){
	printf("%f\n", data.rotY);
	pitch = data.rotY;
	droneState = data.state;
}

/**
* Default constructor
*/
NymeriaCheckObstacle::NymeriaCheckObstacle(){}

/**
* Constructor for the NymeriaCheckObstacle class
* Contains the navdata subscriber, sets the security distance to 100.0 and the speed factor to 1.0 by default
* \param n Node handle for ROS
*/
NymeriaCheckObstacle::NymeriaCheckObstacle(ros::NodeHandle * n){
	int tmpSecurityDist = -1;

	error = 0.0;
	sensorMaxRange = 350.0;

	nh = n;

	sub_navdata = nh->subscribe("/ardrone/navdata", 1, &stateDroneCallback);

	if(nh->hasParam("/nymeriaSecurityDist") && (nh->getParam("/nymeriaSecurityDist", tmpSecurityDist))){

		if(tmpSecurityDist != 100.0){
			ROS_WARN("Current security distance has been overwritten.");
		}

	}

	NymeriaMutexSecurityDistance::lock();
		nh->setParam("nymeriaSecurityDist", 100.0);
	NymeriaMutexSecurityDistance::unlock();

	nh->setParam("nymeriaFactor", 1.0);
}

/**
* Update the distance between the drone and the obstacle, this value is stored in a ROS param named /nymeriaStateObstacle 
* \param cfd Current distance to the obstacle
*/
void NymeriaCheckObstacle::inputCurFrontDist(int cfd){
	int stateObstacle;
	try{
		if(cfd >= 0){

			NymeriaMutexObstacle::lock();
				if(nh->hasParam("/nymeriaStateObstacle")){

					nh->setParam("nymeriaStateObstacle", cfd);
				}
				else throw NymeriaParamExc();
			NymeriaMutexObstacle::unlock();
		}
		else {

			if(nh->getParam("/nymeriaStateObstacle", stateObstacle)){

				if(stateObstacle > 0){
					NymeriaMutexObstacle::lock();
						nh->setParam("nymeriaStateObstacle", -1);
					NymeriaMutexObstacle::unlock();
				}
			}
			else throw NymeriaParamExc();
		}

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr, "%s", error.what());
	}

	// TODO call back

	ros::spinOnce();
	regulation(pitch, 0.1);
	//regulation()
}

/**
 * Getter function for sensor max range,
 * in order to permit the user to retain its current value.
 * @return sensor max range.
 */
double NymeriaCheckObstacle::getSensorMaxRange(){
	return(this->sensorMaxRange);
}

/**
 * Setter function for sensor max range,
 * in order to permit the user to change its value.
 * @param range - sensor max range.
 */
void NymeriaCheckObstacle::setSensorMaxRange(double range){
	this->sensorMaxRange = range;
}

/**
 * Getter function for security distance,
 * in order to permit the user to retain its current value.
 * @return security distance.
 */
double NymeriaCheckObstacle::getSecurityDist(){
	double tmpSecurityDist;
	char nymeriaSecurityDist[] = "/nymeriaSecurityDist";

	nh->getParam(nymeriaSecurityDist, tmpSecurityDist);
	return(tmpSecurityDist);
}

/**
 * Setter function for security distance,
 * in order to permit the user to change its value.
 * @param secDist security distance.
 */
void NymeriaCheckObstacle::setSecurityDist(double secDist){
	try {
		if (secDist >= 0){
			NymeriaMutexSecurityDistance::lock();
				nh->setParam("nymeriaSecurityDist", secDist);
			NymeriaMutexSecurityDistance::unlock();
		}
		else
			throw NymeriaInvalidSecurityDistance();

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr, "%s", error.what());
	}
}

/**
* Regulation method
* Updates the speed factor stored in the ROS param "nymeriaFactor" according to the user orignial command and the estimated pitch of the drone
* @param userCmd original command sent by user. Represented as a double corresponding to a linear speed factor
* @param drone pitch given by sensors on drone
* @return void
*/
void NymeriaCheckObstacle::regulation (double estimatedAngle, double userCmd){

	double tmp_Dist_To_Obstacle(0.0);
	double tmp_SecurityDist(0.0);
	double cmd(0.0);					//speed factor after regulation
	double estimatedCmd(0.0);
	double lastError(0.0);

	nh->getParam("/nymeriaStateObstacle", tmp_Dist_To_Obstacle);
	nh->getParam("/nymeriaSecurityDist", tmp_SecurityDist);

	cmd = pilotage(tmp_Dist_To_Obstacle, tmp_SecurityDist, userCmd);
	
	printf("[NymeriaCheckObstacle::regulation] distance de securité = %f\n", tmp_SecurityDist);

	estimatedCmd = rebouclage(estimatedAngle);

	lastError = error;
	error = cmd - estimatedCmd;

	///// TODO mutex + try catch /////
	//NymeriaMutexObstacle::lock();
	if(nh->hasParam("/nymeriaFactor")){

	  	cmd = PID(lastError, cmd); // get the regulated command 
	  
	  	//tmpSpeedCmd = saturationPente(tmpSpeedCmd);
	  	//saturationCommande(tmpSpeedCmd);

		nh->setParam("nymeriaFactor", cmd);
		printf("[NymeriaCheckObstacle::regulation] Cmd régulée = %f\n", cmd);
	}
		//else throw NymeriaParamExc();
	//NymeriaMutexObstacle::unlock();
}


/**
* First regulataion of the speed factor command regarding the drone distance to obstacle
*
* @param[in] dist_To_Obstacle distance of the drone from the obstacle
* @param[in] securityDist security distance
* @param[in] userCmd initial user command (as speed factor)
* @return regulated command
*/
double NymeriaCheckObstacle::pilotage (	const double& dist_To_Obstacle, const double& securityDist, const double& userCmd	){

	double cmdFactor = (dist_To_Obstacle - securityDist)/100; //new command caculated according to the distance
	saturationCommande(cmdFactor);
	cmdFactor = cmdFactor * userCmd; 
	saturationCommande(cmdFactor);
	return cmdFactor;
}


/**
* PID part of the regulation
* 
* @param[in] lastError
* @return regulated command
*/
double NymeriaCheckObstacle::PID (const double lastError, const double cmd){
	double a0 = 1;
	double a1 = 1;
	double b0 = 0.208;
	double b1 = -0.04901;

	return (b0*lastError/a1) + (b1*error/a1) + cmd;	
}


/**
* Conversion between the pitch of the drone and the speed factor
* 
* @param[in] estimatedAngle drone pitch
* @return speed factor
*/
double NymeriaCheckObstacle::rebouclage(const double& estimatedAngle){
	
	return estimatedAngle * 0.043;
}


/**
* Saturation of the derivative
* 
* @param lastCmd last value of the variable to saturate
* @param currentCmd current value of the variable to saturate
* @param param_saturation set the derivative limit
* @return new saturated value
*/
double NymeriaCheckObstacle::saturationPente(const double lastCmd, const double param_saturation, double& currentCmd)
{	
	if((currentCmd - lastCmd) > param_saturation)
	  	currentCmd += param_saturation;
	
	else if((currentCmd - lastCmd) < -param_saturation)
	  	currentCmd -= param_saturation; 
	
}

/**
* Saturate the value of a variable to 1.0
*
* @param[in, out] cmd, value to saturate
*/
void NymeriaCheckObstacle::saturationCommande(double& cmd)
{
	if(cmd > 1.0)
    	cmd = 1.0;
  	
  	else if (cmd < -1.0)
	    cmd = -1.0;
}
