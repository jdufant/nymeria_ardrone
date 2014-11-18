#include <nymeria_ardrone/NymeriaMutexDrone.h>
#include <iostream> // to get NULL

NymeriaMutexDrone::NymeriaMutexDrone(){};
NymeriaMutexDrone::~NymeriaMutexDrone(){
	instanceFlag = false;
	locked = false;
};

bool NymeriaMutexDrone::instanceFlag = false;
bool NymeriaMutexDrone::locked = false;
NymeriaMutexDrone * NymeriaMutexDrone::mutexDrone = NULL;

NymeriaMutexDrone * NymeriaMutexDrone::getInstance(){
	if(! instanceFlag){
		mutexDrone = new NymeriaMutexDrone();
		instanceFlag = true;
		return mutexDrone;
	}
	else {
		return mutexDrone;
	}
}

void NymeriaMutexDrone::lock(){
	while(locked){}
	locked = true;
}

void NymeriaMutexDrone::unlock(){
	locked = false;
}
