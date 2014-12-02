#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>
#include <iostream> // to get NULL

NymeriaMutexSecurityDistance::NymeriaMutexSecurityDistance(){};

NymeriaMutexSecurityDistance::~NymeriaMutexSecurityDistance(){
  instanceFlag = false;
  locked = false;
};

bool NymeriaMutexSecurityDistance::instanceFlag = false;

bool NymeriaMutexSecurityDistance::locked = false;

NymeriaMutexSecurityDistance * NymeriaMutexSecurityDistance::mutexSecDist = NULL;

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

void NymeriaMutexSecurityDistance::lock(){
  while(locked){}
  locked = true;
}

void NymeriaMutexSecurityDistance::unlock(){
  locked = false;
}
