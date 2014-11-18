#ifndef NYMERIA_MUTEX_DRONE
#define NYMERIA_MUTEX_DRONE

#include <nymeria_ardrone/NymeriaMutex.h>

class NymeriaMutexDrone: public NymeriaMutex{
	public:
		static NymeriaMutexDrone * getInstance();
		static void lock();
		static void unlock();
		~NymeriaMutexDrone();

	private:
		NymeriaMutexDrone();
		static bool locked;
		static bool instanceFlag;
		static NymeriaMutexDrone * mutexDrone;
		
};

#endif