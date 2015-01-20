#include <nymeria_ardrone/NymeriaMutexCommand.h>
#include <iostream>

/**
* Defintion of the class NymeriaMutexCommand, which manages access to the ROS Parameter nymeriaCommand.
**/

/**
* Default constructor in order to create an object of type NymeriaMutexCommand.
**/
NymeriaMutexCommand::NymeriaMutexCommand(){};

/**
 * Destructor resetting class attributes.
 */
NymeriaMutexCommand::~NymeriaMutexCommand(){
	instanceFlag = false;
	locked = false;
};

/**
 * Flag in order to make sure there is only one instance of the class.
 * true - has been already instantiated once.
 * false - hasn't been instantiated yet.
 */
bool NymeriaMutexCommand::instanceFlag = false;
/**
 * Attribute that marks whether or not the resource has been acquired yet.
 * true - has been already acquired.
 * false - hasn't been acquired yet.
 */
bool NymeriaMutexCommand::locked = false;

/**
 * First declaration of instance of type NymeriaMutex.
 */
NymeriaMutexCommand * NymeriaMutexCommand::mutexDrone = NULL;

/**
 * Function in order to get an instance of NymeriaMutexCommand.
 * @return useful, i.e. not NULL object of type NymeriaMutex.
 */
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

/**
 * Method in order to lock or acquire resource.
 * Resource can not be acquired by any other object while being locked.
 */
void NymeriaMutexCommand::lock(){
	while(locked){}
	locked = true;
}

/**
 * Method in order to unlock or release resource.
 * Resource can be acquired by an other object after being released.
 */
void NymeriaMutexCommand::unlock(){
	locked = false;
}
