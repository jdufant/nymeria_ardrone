#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <iostream> // to get NULL

NymeriaMutexObstacle::NymeriaMutexObstacle(){};
NymeriaMutexObstacle::~NymeriaMutexObstacle(){
	instanceFlag = false;
	locked = false;
};

bool NymeriaMutexObstacle::instanceFlag = false;
bool NymeriaMutexObstacle::locked = false;
NymeriaMutexObstacle * NymeriaMutexObstacle::mutexObstacle = NULL;

NymeriaMutexObstacle * NymeriaMutexObstacle::getInstance(){
	if(! instanceFlag){
		mutexObstacle = new NymeriaMutexObstacle();
		instanceFlag = true;
		return mutexObstacle;
	}
	else {
		return mutexObstacle;
	}
}

void NymeriaMutexObstacle::lock(){
	while(locked){}
	locked = true;
}

void NymeriaMutexObstacle::unlock(){
	locked = false;
}
