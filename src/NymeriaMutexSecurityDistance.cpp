#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>
#include <iostream>

/**
* Defintion of the class NymeriaMutexSecurityDistance, which manages access to the ROS Parameter nymeriaSecurityDistance.
**/

/**
* Default constructor in order to create an object of type NymeriaMutexSecurityDistance.
**/
NymeriaMutexSecurityDistance::NymeriaMutexSecurityDistance(){};

/**
* Destructor resetting class attributes.
*/
NymeriaMutexSecurityDistance::~NymeriaMutexSecurityDistance(){
  instanceFlag = false;
  locked = false;
};


/**
* Flag in order to make sure there is only one instance of the class.
* true - has been already instantiated once.
* false - hasn't been instantiated yet.
*/
bool NymeriaMutexSecurityDistance::instanceFlag = false;
/**
* Attribute that marks whether or not the resource has been acquired yet.
* true - has been already acquired.
* false - hasn't been acquired yet.
*/
bool NymeriaMutexSecurityDistance::locked = false;

/**
* First declaration of instance of type NymeriaMutexSecurityDistance.
*/
NymeriaMutexSecurityDistance * NymeriaMutexSecurityDistance::mutexSecDist = NULL;

/**
* Function in order to get an instance of NymeriaMutexSecurityDistance.
* @return useful, i.e. not NULL object of type NymeriaMutexSecurityDistance.
*/
NymeriaMutexSecurityDistance * NymeriaMutexSecurityDistance::getInstance(){
  if(! instanceFlag){
    mutexSecDist = new NymeriaMutexSecurityDistance();
    instanceFlag = true;
    return mutexSecDist;
  }
  else {
    return mutexSecDist;
  }
}

/**
* Method in order to lock or acquire resource.
* Resource can not be acquired by any other object while being locked.
*/
void NymeriaMutexSecurityDistance::lock(){
  while(locked){}
  locked = true;
}

/**
* Method in order to unlock or release resource.
* Resource can be acquired by an other object after being released.
*/
void NymeriaMutexSecurityDistance::unlock(){
  locked = false;
}
