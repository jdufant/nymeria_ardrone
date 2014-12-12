#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>

NymeriaCheckObstacle::NymeriaCheckObstacle(){}
NymeriaCheckObstacle::NymeriaCheckObstacle(ros::NodeHandle * n, int securityDist){
	int tmpSecurityDist = -1;

	sommeError = 0;
	error = 0;
	cmdEstimePrec = 0;
	angleEstimePrec = 0;
	angleEstimePrec2 = 0;
	cmd = 0;

	nh = n;
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
			nh->setParam("nymeriaFactor", 0.0);
		//NymeriaMutexObstacle::unlock();




	}
	catch(NymeriaExceptions& error){
		/* Display error message. */
		fprintf(stderr,error.what());
	}

}

void NymeriaCheckObstacle::inputCurFrontDist(int cfd){
	int stateObstacle;
	try{
		if(cfd >= 0){

			NymeriaMutexObstacle::lock();
				if(nh->hasParam("/nymeriaStateObstacle")){

					nh->setParam("/nymeriaStateObstacle", cfd);
				}
				else throw NymeriaParamExc();
			NymeriaMutexObstacle::unlock();
		}
		else {

			if(nh->getParam("/nymeriaStateObstacle", stateObstacle)){

				if(stateObstacle > 0){
					NymeriaMutexObstacle::lock();
						nh->setParam("/nymeriaStateObstacle", -1);
					NymeriaMutexObstacle::unlock();
				}
			}
			else throw NymeriaParamExc();
		}

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr,error.what());
	}

	// TODO call back

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
			nh->setParam("/nymeriaSecurityDist", securityDist);
			NymeriaMutexSecurityDistance::unlock();
		}
		else throw NymeriaParamExc();

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr,error.what());
	}
	
}



// TODO not valid for doxygen
/*
* Pilote le drone
*
* @param tmpStateObstacle, retourné par le capteur
* @param tmpFactor, cmd de l'utilisateur en pourcentage de vitesse max
* @param tmpSecurityDist, distance limite à l'obstacle
* @param angleEstime, angle retourné par les infos du drone
* @return cmd
*/
void NymeriaCheckObstacle::regulation (double angleEstime){
	double tmpStateObstacle;
	double tmpSecurityDist;
	double tmpFactor;

	nh->getParam("/nymeriaStateObstacle", tmpStateObstacle);
	nh->getParam("/nymeriaSecurityDist", tmpSecurityDist);
	nh->getParam("/nymeriaFactor", tmpFactor);
	
	//régule la cmd en fct de la distance
	double cmdPrec = cmd; // init
	cmd = pilotage();
	cmd = saturationPente(cmdPrec);
	
	//convertie l'angle en cmd estimé
	angleEstimePrec2 = angleEstimePrec;
	angleEstimePrec = angleEstime;
	double cmdEstime = rebouclage(angleEstime);
	cmdEstimePrec = cmdEstime;
	
	//rebouclage avec regulation et retourne la cmd regulé
	double errorPrec = error;
	error = cmd - cmdEstime;
	sommeError += error;

	// TODO mutex + try catch
	//NymeriaMutexObstacle::lock();
	if(nh->hasParam("/nymeriaFactor")){
		nh->setParam("/nymeriaFactor", PID(errorPrec));
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
double NymeriaCheckObstacle::pilotage (){

	double tmpStateObstacle;
	double tmpSecurityDist;
	double tmpFactor;

	nh->getParam("/nymeriaStateObstacle", tmpStateObstacle);
	nh->getParam("/nymeriaSecurityDist", tmpSecurityDist);
	nh->getParam("/nymeriaFactor", tmpFactor);

	double cmd = tmpStateObstacle - tmpSecurityDist;
	if(cmd > 1.0){
		cmd = 1.0;
	} else if (cmd < -1.0){
		cmd = -1.0;
	}
	cmd = cmd * tmpFactor / 100.0;
	if(cmd > 1.0){
		cmd = 1.0;
	} else if (cmd < -1.0){
		cmd = -1.0;
	}
	return cmd;
}

/*
* Réalise la régulation de la commande
* 
* @param errorPrec
* @return la valeur à donner en commande au drone
*/
double NymeriaCheckObstacle::PID (double errorPrec){
	double Kp = 0.1105;
	double Ki = 1.5935;
	double Kd = 0.2541;
	
	//commande = Kp*P + Ki*I + Kd*D	
	//commande = Kp * erreur + Ki * somme_erreurs + Kd * (erreur - erreur_précédente)
	return Kp*error + Ki*sommeError + Kd*(error - errorPrec);
}

/*
* Calcule la cmd estimé pour asservir la cmd
* 
* @param angleEstime, angle retourné par les infos du drone
* @return cmdEstime, angle convertie en cmd pour le rebouclage
*/
double NymeriaCheckObstacle::rebouclage(double angleEstime){
	double a0 = 0.05191;
	double a1 = 0.1217;
	double a2 = 0.07727;
	double b0 = -0.824;
	double b1 = 1.0;
	
	return angleEstimePrec2*a2/b0 + angleEstimePrec*a1/b0 + angleEstime*a0/b0 - cmdEstimePrec*b1/b0;
}

/*
* Saturation de la pente de commande
* 
* @param cmd, cmd a saturé
* @return cmd, cmd saturé
*/
double NymeriaCheckObstacle::saturationPente(double cmdPrec){
	double param = 0.5; //paramètre de saturation de la pente
	if((cmd - cmdPrec) > param){
		return param;
	} else if((cmd - cmdPrec) < -param){
		return -param;
	} else {
		return cmd;
	}
}
