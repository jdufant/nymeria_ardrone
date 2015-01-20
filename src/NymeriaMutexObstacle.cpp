#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <iostream>

/**
* Defintion of the class NymeriaMutexObstacle, which manages access to the ROS Parameter nymeriaStateObstacle.
**/

/**
* Default constructor in order to create an object of type NymeriaStateObstacle.
**/
NymeriaMutexObstacle::NymeriaMutexObstacle(){};

/**
* Destructor resetting class attributes.
*/
NymeriaMutexObstacle::~NymeriaMutexObstacle(){
	instanceFlag = false;
	locked = false;
};

/**
* Flag in order to make sure there is only one instance of the class.
* true - has been already instantiated once.
* false - hasn't been instantiated yet.
*/
bool NymeriaMutexObstacle::instanceFlag = false;
/**
* Attribute that marks whether or not the resource has been acquired yet.
* true - has been already acquired.
* false - hasn't been acquired yet.
*/
bool NymeriaMutexObstacle::locked = false;
/**
* First declaration of instance of type NymeriaMutex.
*/
NymeriaMutexObstacle * NymeriaMutexObstacle::mutexObstacle = NULL;

/**
* Function in order to get an instance of NymeriaMutexObstacle.
* @return useful, i.e. not NULL object of type NymeriaMutex.
*/
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

/**
* Method in order to lock or acquire resource.
* Resource can not be acquired by any other object while being locked.
*/
void NymeriaMutexObstacle::lock(){
	while(locked){}
	locked = true;
}

/**
* Method in order to unlock or release resource.
* Resource can be acquired by an other object after being released.
*/
void NymeriaMutexObstacle::unlock(){
	locked = false;
}
