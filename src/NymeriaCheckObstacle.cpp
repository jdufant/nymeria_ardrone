#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>

NymeriaCheckObstacle::NymeriaCheckObstacle(){}
NymeriaCheckObstacle::NymeriaCheckObstacle(ros::NodeHandle * n, int securityDist){
	int tmpSecurityDist = -1;

	nh = n;
	this->securityDist = securityDist;

	try {
		if(securityDist >= 0){
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
