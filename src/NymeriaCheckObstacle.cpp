#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>

double pitch = 0.0;
int droneState = 0;

void stateDroneCallback(const ardrone_autonomy::Navdata& data){
	printf("%f\n", data.rotY);
	pitch = data.rotY;
	droneState = data.state;
}

NymeriaCheckObstacle::NymeriaCheckObstacle(){}
NymeriaCheckObstacle::NymeriaCheckObstacle(ros::NodeHandle * n, int securityDist){
	int tmpSecurityDist = -1;

	//sumError = 0.0;
	error = 0.0;
	//lastCmdEstimated = 0.0;
	//lastAngleEstimated = 0.0;
	//lastAngleEstimated2 = 0.0;
	//cmd = 0.0;

	nh = n;

	sub_navdata = nh->subscribe("/ardrone/navdata", 1, &stateDroneCallback);

	this->securityDist = securityDist;

	try {
	  if(securityDist >= 0.0){

	    if(nh->hasParam("/nymeriaSecurityDist") && (nh->getParam("/nymeriaSecurityDist", tmpSecurityDist))){
	      
	      if(tmpSecurityDist != securityDist){
		ROS_WARN("Given security distance does not match security distance given in Nymeria.");
		ROS_WARN("First security distance given will be considered.");
	      }

	    }
	    else {
	      NymeriaMutexSecurityDistance::lock();
	      nh->setParam("nymeriaSecurityDist", securityDist);
	      NymeriaMutexSecurityDistance::unlock();
	    }
	    
	  }
	  else
	    throw NymeriaInvalidSecurityDistance();

	  //NymeriaMutexObstacle::lock();
	  nh->setParam("nymeriaFactor", 1.0);
	  //NymeriaMutexObstacle::unlock();




	}
	catch(NymeriaExceptions& error){
		/* Display error message. */
		fprintf(stderr, "%s", error.what());
	}

}

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

int NymeriaCheckObstacle::getSecurityDist(){
	return securityDist;
}

void NymeriaCheckObstacle::setSecurityDist(int sd){
	securityDist = sd;
	try {
		if(nh->hasParam("/nymeriaSecurityDist")){
			NymeriaMutexSecurityDistance::lock();
			nh->setParam("nymeriaSecurityDist", securityDist);
			NymeriaMutexSecurityDistance::unlock();
		}
		else throw NymeriaParamExc();

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr, "%s", error.what());
	}
	
}


/**
* Pilote le drone
*
* @param tmpStateObstacle, retourné par le capteur
* @param tmpFactor, cmd de l'utilisateur en pourcentage de vitesse max
* @param tmpSecurityDist, distance limite à l'obstacle
* @param angleEstimated, angle retourné par les infos du drone
* @return cmd
*/
void NymeriaCheckObstacle::regulation (double angleEstimated, double userCmd){
	double tmp_Dist_To_Obstacle(0.0);
	double tmp_SecurityDist(0.0);
	double cmd(0.0);			//speed factor after regulation
	double estimatedCmd(0.0);
	double lastError(0.0);

	nh->getParam("/nymeriaStateObstacle", tmp_Dist_To_Obstacle);
	nh->getParam("/nymeriaSecurityDist", tmp_SecurityDist);
	
	//régule la cmd en fct de la distance
	//double lastCmd = cmd; // init

	cmd = pilotage(tmp_Dist_To_Obstacle, tmp_SecurityDist, userCmd);
	
	//cmd = saturationPente(lastCmd);
	//saturationCommande(cmd);
	
	printf("[NymeriaCheckObstacle::regulation] distance de securité = %f\n", tmp_SecurityDist);
	
	//convertie l'angle en cmd estimé
	//lastAngleEstimated2 = lastAngleEstimated;
	//lastAngleEstimated = angleEstimated;

	estimatedCmd = rebouclage(angleEstimated);

	//lastCmdEstimated = cmdEstimated;
	
	//rebouclage avec regulation et retourne la cmd regulé
	lastError = error;
	error = cmd - estimatedCmd;
	
	//sumError += error;

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


/*
* Calcule la commande du drone en fonction de la cmd user et de la distance de l'obstacle et de la limite
*
* @param distance, retourné par le capteur
* @param cmd_user, cmd de l'utilisateur en pourcentage de vitesse max
* @param limite, distance limite à l'obstacle
* @return cmd
*/
double NymeriaCheckObstacle::pilotage (	const double& dist_To_Obstacle, const double& securityDist, const double& userCmd	){

	double cmdFactor = (dist_To_Obstacle - securityDist)/100; //new command caculated according to the distance
	saturationCommande(cmdFactor);
	cmdFactor = cmdFactor * userCmd; 
	saturationCommande(cmdFactor);
	return cmdFactor;
}


/*
* Réalise la régulation de la commande
* 
* @param lastError
* @return la valeur à donner en commande au drone
*/
double NymeriaCheckObstacle::PID (const double lastError, const double cmd){
	double a0 = 1;
	double a1 = 1;
	double b0 = 0.208;
	double b1 = -0.04901;

	return (b0*lastError/a1) + (b1*error/a1) + cmd;	
}


/*
* Calcule la cmd estimé pour asservir la cmd
* 
* @param angleEstimated, angle retourné par les infos du drone
* @return cmdEstimated, angle convertie en cmd pour le rebouclage
*/
double NymeriaCheckObstacle::rebouclage(const double& angleEstimated){
	
	return angleEstimated * 0.043;
}


/*
* Saturation de la pente de commande
* 
* @param cmd, cmd a saturé
* @return cmd, cmd saturé
*/
double NymeriaCheckObstacle::saturationPente(const double lastCmd, const double param_saturation, double& currentCmd)
{	
	if((currentCmd - lastCmd) > param_saturation)
	  	currentCmd += param_saturation;
	
	else if((currentCmd - lastCmd) < -param_saturation)
	  	currentCmd -= param_saturation; 
	
}

void NymeriaCheckObstacle::saturationCommande(double& cmd)
{
	if(cmd > 1.0)
    	cmd = 1.0;
  	
  	else if (cmd < -1.0)
	    cmd = -1.0;
}
