#include <nymeria_ardrone/NymeriaMutexCommand.h>
#include <iostream> // to get NULL

NymeriaMutexCommand::NymeriaMutexCommand(){};
NymeriaMutexCommand::~NymeriaMutexCommand(){
	instanceFlag = false;
	locked = false;
};

bool NymeriaMutexCommand::instanceFlag = false;
bool NymeriaMutexCommand::locked = false;
NymeriaMutexCommand * NymeriaMutexCommand::mutexDrone = NULL;

NymeriaMutexCommand * NymeriaMutexCommand::getInstance(){
	if(! instanceFlag){
		mutexDrone = new NymeriaMutexCommand();
		instanceFlag = true;
		return mutexDrone;
	}
	else {
		return mutexDrone;
	}
}

void NymeriaMutexCommand::lock(){
	while(locked){}
	locked = true;
}

void NymeriaMutexCommand::unlock(){
	locked = false;
}
