#ifndef NYMERIA_PARAM_EXC_H
#define NYMERIA_PARAM_EXC_H

#include <nymeria_ardrone/NymeriaExceptions.h>

/**
 * Declaration of the class NymeriaParamExc, that declares the exception
 * thrown when the ROS parameter requested does not exist or was misspelled.
 */

class NymeriaParamExc :
	public NymeriaExceptions
{
public:
	NymeriaParamExc(string msg = "");
	virtual ~NymeriaParamExc(void) throw();
	virtual const char * what() const throw();
private:

};

#endif