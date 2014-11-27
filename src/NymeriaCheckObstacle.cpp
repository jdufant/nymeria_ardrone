#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>

NymeriaCheckObstacle::NymeriaCheckObstacle(){}
NymeriaCheckObstacle::NymeriaCheckObstacle(ros::NodeHandle * n, int securityDist){
	int tmpSecurityDist = -1;

	nh = n;
	this->securityDist = securityDist;

	if(nh->getParam("/nymeria_ardrone/securityDist", tmpSecurityDist)){
		if(tmpSecurityDist != securityDist){
			ROS_WARN("Given security distance does not match security distance given in Nymeria.");
			ROS_WARN("First security distance given will be considered.");
			securityDist = tmpSecurityDist;
		}
	}
	else {
		// NymeriaMutexObstacle::lock();
		nh->setParam("/nymeria_ardrone/securityDist", securityDist);
		// NymeriaMutexObstacle::unlock();
	}
}

void NymeriaCheckObstacle::inputCurFrontDist(int cfd){
	int stateObstacle;
	try{
		if((cfd < securityDist) && (cfd >= 0)){

			NymeriaMutexObstacle::lock();
				if(nh->hasParam("/nymeria_ardrone/stateObstacle")){

					nh->setParam("/nymeria_ardrone/stateObstacle", cfd);
				}
				else throw NymeriaParamExc();
			NymeriaMutexObstacle::unlock();
		}
		else {

			if(nh->getParam("/nymeria_ardrone/stateObstacle", stateObstacle)){

				if(stateObstacle > 0){
					NymeriaMutexObstacle::lock();
						nh->setParam("/nymeria_ardrone/stateObstacle", -1);
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
		if(nh->hasParam("/nymeria_ardrone/securityDist")){
			// NymeriaMutexObstacle::lock();
			nh->setParam("/nymeria_ardrone/securityDist", securityDist);
			// NymeriaMutexObstacle::unlock();
		}
		else throw NymeriaParamExc();

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr,error.what());
	}
	
}