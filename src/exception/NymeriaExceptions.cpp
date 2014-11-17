#include <nymeria_ardrone/NymeriaExceptions.h>

/**
 * Definition of the class NymeriaExceptions, that defines the base class
 * for all exceptions particular to Nymeria.
 */

NymeriaExceptions::NymeriaExceptions(string msg)
{
	this->errMsg = msg;
}

NymeriaExceptions::~NymeriaExceptions(void){}

/* Overriding what() function from standard Exception. */
const char * NymeriaExceptions::what() const throw(){
	/* display error message */
	return this->errMsg.c_str();
}
