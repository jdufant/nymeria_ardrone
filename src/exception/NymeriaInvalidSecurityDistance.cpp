#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>

/**
 * Definition of the class NymeriaInvalidSecurityDistance, that defines the exception
 * thrown when the an invalid security distance is entered.
 */

NymeriaInvalidSecurityDistance::NymeriaInvalidSecurityDistance(void) : NymeriaExceptions(""){}


NymeriaInvalidSecurityDistance::~NymeriaInvalidSecurityDistance(void) throw(){}

/* overriding what() for particular error message. */
const char * NymeriaInvalidSecurityDistance::what() const throw()
{
	string s = "Invalid security distance.\n Security distance must be an integer greater or equal to 0 cm.\n";
	return s.c_str();
}
