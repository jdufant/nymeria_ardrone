#ifndef NYMERIA_MUTEX_OBSTACLE_H
#define NYMERIA_MUTEX_OBSTACLE_H

#include <nymeria_ardrone/NymeriaMutex.h>

class NymeriaMutexObstacle: public NymeriaMutex{
	public:
		static NymeriaMutexObstacle * getInstance();
		static void lock();
		static void unlock();
		~NymeriaMutexObstacle();

	private:
		NymeriaMutexObstacle();
		static bool locked;
		static bool instanceFlag;
		static NymeriaMutexObstacle * mutexObstacle;
		
};

#endif