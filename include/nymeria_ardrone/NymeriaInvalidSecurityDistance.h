#ifndef NYMERIA_INVALID_SECURITY_DISTANCE_EXC_H
#define NYMERIA_INVALID_SECURITY_DISTANCE_EXC_H

#include <nymeria_ardrone/NymeriaExceptions.h>

/**
 * Declaration of the class NymeriaParamExc, that declares the exception
 * thrown when the ROS parameter requested does not exist or was misspelled.
 */

class NymeriaInvalidSecurityDistance :
	public NymeriaExceptions
{
public:
	NymeriaInvalidSecurityDistance(void);
	virtual ~NymeriaInvalidSecurityDistance(void) throw();
	virtual const char * what() const throw();
};

#endif