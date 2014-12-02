#ifndef NYMERIA_SECURITY_DISTANCE_H
#define NYMERIA_SECURITY_DISTANCE_H
#include <nymeria_ardrone/NymeriaMutex.h>

class NymeriaMutexSecurityDistance: public NymeriaMutex{
public:
  static NymeriaMutexSecurityDistance * getInstance();
  static void lock();
  static void unlock();
  ~NymeriaMutexSecurityDistance();
private:
  NymeriaMutexSecurityDistance();
  static bool locked;
  static bool instanceFlag;
  static NymeriaMutexSecurityDistance * mutexSecDist;
};
#endif
