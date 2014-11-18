#ifndef NYMERIA_EXCEPTIONS_H
#define NYMERIA_EXCEPTIONS_H

#include <exception>
#include <string>
using namespace std;

/**
 * Declaration of the class NymeriaExceptions, that declares the base class
 * for all exceptions particular to Nymeria.
 */

class NymeriaExceptions : public exception{
	public:
		NymeriaExceptions(string msg);
		virtual ~NymeriaExceptions(void) throw();
		virtual const char * what() const throw();
	private:
		string errMsg;
};

#endif