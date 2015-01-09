#include "UDPWrapper.h"

#include <stdio.h>   /* Standard input/output definitions */
//#include <signal.h>   /* Standard signals definitions */
//#include <string.h>  /* String function definitions */
//#include <unistd.h>  /* UNIX standard function definitions */
#include <errno.h>   /* Error number definitions */
#include <stdlib.h>
//#include <stdint.h>


#define BUFFER_SIZE 8
#define NB_VAL 10
#define TIMEOUT 1000000

int main()
{
  int cpt_boucle = 0;
  int timeoutCnt = 0;
  int pgExit = 0;

  int bsent, chread;
  char readBuffer[BUFFER_SIZE];
  char sendBuffer[4];
 
  printf("tcsetattr succeed\nIMPALA version\n");

	// Open UDP server //
	UDPServer server("192.168.1.1", 7777);

	printf("Wait for client to be ready\n");
	// wait for client to connect
	while(server.recv(readBuffer, BUFFER_SIZE) == -1);
	
	printf("connected\n");
	
	while (cpt_boucle < 500 && pgExit == 0) {

		printf("test %d\n", cpt_boucle);
	  	sprintf(sendBuffer, "%d", cpt_boucle);
	  	bsent = server.send(sendBuffer, 4);
	  	cpt_boucle ++;
	  	
	  	//printf("char = %d\n", server.recv(readBuffer, BUFFER_SIZE));
	  	
	  	try {
	  		server.recv(readBuffer, BUFFER_SIZE);
	  	}

	  	catch (const std::exception & ex) {
	  		printf(ex.what());
	  		break;
	  	}
	    
	} //end while
	

	printf("Program exit...\n");

} //end main
