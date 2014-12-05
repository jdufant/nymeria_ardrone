#ifndef NYMERIA_MUTEX_COMMAND_H
#define NYMERIA_MUTEX_COMMAND_H

#include <nymeria_ardrone/NymeriaMutex.h>

class NymeriaMutexCommand: public NymeriaMutex{
	public:
		static NymeriaMutexCommand * getInstance();
		static void lock();
		static void unlock();
		~NymeriaMutexCommand();

	private:
		NymeriaMutexCommand();
		static bool locked;
		static bool instanceFlag;
		static NymeriaMutexCommand * mutexDrone;
		
};

#endif