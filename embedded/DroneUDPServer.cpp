#include "UDPWrapper.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <signal.h>   /* Standard signals definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <sstream>


#define BUFFER_SIZE 8
#define NB_VAL 10 		// number for a mean value
#define traitMargin 60 	// margin for filtering data in "traitement()"

/**
* \file DroneUDPServer
*/


int traitTab(int* tab, int size);

/**
* Reads the buffer char by char to detect a specific delimitation char
* this delimitation char and all char after it are replaced by the null char in the read buffer
* \param[in] fd file descriptor
* \param[in, out] buf read buffer
* \param[in] until delimitation char
* \return number of char in the buffer before
*/
int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0; int n=0; int chread=0;
    
    do { 
        n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
	  		usleep(1000); // wait 1 msec, then try again
	  		continue;
        }
        buf[i] = b[0]; 
		i++;

    } while( b[0] != until);
    
    chread = i-1;

    for (int j = (i-1); i < BUFFER_SIZE; i++)
      {
	buf[j] = '\0';  // null char for the rest of the buffer
      }

    return chread;
}

/**
* Configure the serial port to read data
* baudrate = 115200, 8N1, 
void setSerialOptions( struct termios& options)
{
  	/* Set Baud Rate */
	cfsetispeed( &options, B115200 );
	//cfsetospeed( &options, B115200 );

	    options.c_cflag |= ( CLOCAL | CREAD );
	    // Set the Charactor size
	    options.c_cflag &= ~CSIZE; /* Mask the character size bits */
	    options.c_cflag |= CS8;    /* Select 8 data bits */
	    // Set parity - No Parity (8N1)
	    options.c_cflag &= ~PARENB;
	    options.c_cflag &= ~CSTOPB;
	    // Disable Hardware flowcontrol
	    //options.c_cflag &= ~CNEW_RTSCTS;  //-- not supported
	    // Enable Raw Input
	    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	    // Disable Software Flow control
	    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    }


int main()
{
  std::stringstream ss;
  std::string tmpStr;
  int cpt_tab = 0;
  int cpt_boucle = 0;

  int bsent, chread;
  int fd, sockfd; /* File descriptor for the port */

  int tabVal[NB_VAL];
  char readBuffer[BUFFER_SIZE];
  char sendBuffer[4];
  int value, tmp_value = 0;

  struct termios options;

  /*Open serial port
    try to open ttyUSB0 first. If failed, try to open ttyUSB1
  */
	fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);

	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyUSB0");
		printf("trying to open /dev/ttyUSB1\n");
		fd = open("/dev/ttyUSB1", O_RDONLY | O_NOCTTY | O_NDELAY);
		if (fd == -1) {
			perror("open_port: Unable to open /dev/ttyUSB1");
		}
		else{
		  //if the serial port opened successfully, the next command reads the port with no delay option
		  fcntl(fd, F_SETFL, 0);
		  printf("ttyUSB1 successfully opened\n");
		}
	}
	else {
		fcntl(fd, F_SETFL, 0);
		printf("ttyUSB0 successfully opened\n");
	}

	tcgetattr( fd, &options );

	setSerialOptions(options);

	if ( tcsetattr( fd, TCSANOW, &options ) == -1 )
		perror("Error with tcsetattr");
	else
		printf("tcsetattr succeed\nGODZILLA version.\n");

	// Open UDP server //
	UDPServer server("192.168.1.1", 7777);

	printf("Wait for client to be ready\n");
	// wait for client to connect
	while(server.recv(readBuffer, BUFFER_SIZE) == -1);
	
	printf("connected\n");
	while (1) {

		// Read from the serial port
		// TODO : check response time (maybe with baud rate) and read() returned value is 0 for EOF
		chread = serialport_read_until(fd, readBuffer, 'x');
		  
		if (chread > 0) {

		    if((tabVal[cpt_tab] = atoi(readBuffer)) < 380){
		      //printf("recu : %s\n", readBuffer);
		      cpt_tab ++;
		    }

		    cpt_boucle++;
		}

		if (cpt_boucle>=NB_VAL && cpt_tab>0){
		    value = traitTab(tabVal, cpt_tab);
		    printf("%d tabtrait => %d\n",cpt_tab, value);
		    sprintf(sendBuffer, "%d", value);
		    bsent = server.send(sendBuffer, 4);

		    cpt_boucle = 0;
		    cpt_tab = 0;

		}	

	  	/*try {
	  		server.recv(readBuffer, BUFFER_SIZE);
	  	}

	  	catch (const std::exception & ex) {
	  		printf(ex.what());
		break;
	  	}*/

	} //end while
	
	printf("Closed\n");

	close(fd);
}
/**
* Filtering the value of an array
*
*/
int traitTab(int* tab, int size)
{
  int total = 0;
  int meanVal = 0;
  const int ecart = 20;

  for(int i =0; i < size; i++) {
    //printf("tab = %d\n", tab[i]);
    total += tab[i];
  }

  meanVal = total/size;
  //printf("meanVal intermédiaire = %d\n", meanVal);

  for(int i =0; i < size; i++){
    if (abs(tab[i] - meanVal) > traitMargin){
      for (int j = i; j < size-1; j++)
	tab[j] = tab[j+1];
      size--;
    }
  }

  total = 0;
  for(int i =0; i < size; i++) {
    total += tab[i];
    //printf("tab[%d] = %d\n", i, tab[i]);
  }

  //printf("total = %d\nsize = %d\n", total, size);
  
  return (int)total/size;
}
