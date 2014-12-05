#include <nymeria_ardrone/NymeriaParamExc.h>

/**
 * Definition of the class NymeriaParamExc, that defines the exception
 * thrown when the ROS parameter requested does not exist or was misspelled.
 */

NymeriaParamExc::NymeriaParamExc(string msg) : NymeriaExceptions(msg){}


NymeriaParamExc::~NymeriaParamExc(void) throw(){}

/* overriding what() for particular error message. */
const char * NymeriaParamExc::what() const throw()
{
	string s = "ROS parameter not existent or misspelled.";
	return s.c_str();
}
