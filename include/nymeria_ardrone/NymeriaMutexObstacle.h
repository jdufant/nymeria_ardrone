#ifndef NYMERIA_MUTEX_OBSTACLE
#define NYMERIA_MUTEX_OBSTACLE

#include <nymeria_ardrone/NymeriaMutex.h>

class NymeriaMutexObstacle: public NymeriaMutex{
	public:
		NymeriaMutexObstacle();
		// static bool nymeriaTryLock();
		// static void nymeriaUnlock();
};

#endif