#include <nymeria_ardrone/NymeriaParamExc.h>

/**
 * Definition of the class NymeriaParamExc, that defines the exception
 * thrown when the ROS parameter requested does not exist or was misspelled.
 */

/**
 * Constructor in order to create an object of type NymeriaParamExc.
 * @param msg - message to be shown, when exception is thrown.
 */
NymeriaParamExc::NymeriaParamExc(string msg) : NymeriaExceptions(msg){}

/**
 * Method in order to throw exception.
 */
NymeriaParamExc::~NymeriaParamExc(void) throw(){}

/* overriding what() for particular error message. */
const char * NymeriaParamExc::what() const throw()
{
	string s = "ROS parameter not existent or misspelled.";
	return s.c_str();
}
