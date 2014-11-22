#include <nymeria_ardrone/UDPWrapper.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */


int main()
{
	int bytes, bsent;
	int nbMsg = 10;
	char msg[100];
	char stringTest[3];
	
	printf("Hello world\n");

	// Open UDP server //

	UDPServer leServer("127.0.0.1", 7777);
	// UDPServer leServer("192.168.1.1", 7777);
	
	printf("connected\n");

	leServer.recv(msg, 100);

	printf("send : return = %d\n", leServer.send("oest",4));

	
	while (1) {
		// Read from the port
		usleep(1000000);

		stringTest[0] = '1';
		//stringTest[1] = '2';
		//stringTest[2] = '3';

		//perror ("read error:");

		bsent = leServer.send(stringTest, 10);
		nbMsg--;
	}
}