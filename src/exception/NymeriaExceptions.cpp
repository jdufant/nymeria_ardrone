#include <nymeria_ardrone/NymeriaExceptions.h>

/**
 * Definition of the class NymeriaExceptions, that defines the base class
 * for all exceptions particular to Nymeria.
 */
 
 /**
* \file NymeriaExceptions.cpp
* \class NymeriaExceptions NymeriaExceptions.h
* \author Team-Nymeria
* \version 0.2
* \date 18th of January 2015
*/

/**
 * Constructor in order to create an object of type NymeriaException.
 **/
NymeriaExceptions::NymeriaExceptions(string msg){
	this->errMsg = msg;
}

NymeriaExceptions::~NymeriaExceptions(void) throw(){}

/**
 * Overriding what() function from standard Exception.
 **/
const char * NymeriaExceptions::what() const throw(){
	/* display error message */
	return this->errMsg.c_str();
}
