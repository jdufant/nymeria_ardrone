#include "UDPWrapper.h"

#include <stdio.h>   /* Standard input/output definitions */
#include <signal.h>   /* Standard signals definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */

int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0; int n =0;
    do { 
        n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 2 * 1000 ); // wait 1 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until);

    buf[i] = 0;  // null terminate the string
    return i;
}

int main()
{
        int bsent, chread;
	int fd, sockfd; /* File descriptor for the port */

	char buffer[10];
	char *bufptr;

	struct termios options;

	fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);
	if (fd == -1) {
		perror("open_port: Unable to open /dev/ttyUSB0");
		printf("trying to open /dev/ttyUSB1\n");
		fd = open("/dev/ttyUSB1", O_RDONLY | O_NOCTTY | O_NDELAY);
		if (fd == -1) {
			perror("open_port: Unable to open /dev/ttyUSB1");
		}
		else{
			fcntl(fd, F_SETFL, 0);

		}
	}
	else {
		fcntl(fd, F_SETFL, 0);
	}

	tcgetattr( fd, &options );

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
	// Chose raw (not processed) output
	//options.c_oflag &= ~OPOST;

	if ( tcsetattr( fd, TCSANOW, &options ) == -1 )
		perror("Error with tcsetattr");
	else
		printf("tcsetattr succeed\n");

	// Open UDP server //
	UDPServer server("192.168.1.1", 7777);

	printf("Wait for client to be ready\n");
	// wait for client to connect
	while(server.recv(buffer, 10) == -1);
	
	while (1) {
	  // Read from the port
	  // TODO : check response time (maybe with baud rate) and read() returned value is 0 for EOF

	  chread = serialport_read_until(fd, buffer, 'x');
	  //bufptr = strcpy(bufptr, buffer);
	  printf("recu %d char : %s\n", chread, buffer);
	  bsent = server.send(buffer, 10);
		
	}

	close(fd);
}
