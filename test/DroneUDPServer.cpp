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
    int i=0;
    do { 
        int n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
            usleep( 2 * 1000 ); // wait 1 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until);

    buf[i] = 0;  // null terminate the string
    return 0;
}

int main()
{
	int bytes, bsent;
	char msg[10];
	int fd, sockfd; /* File descriptor for the port */

	char buffer[5];
	char *bufptr;

	struct termios options;

	fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);
	//fd = open("/dev/tty.usbserial-14P53099", O_RDONLY | O_NOCTTY | O_NDELAY);
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
	cfsetispeed( &options, B9600 );
	cfsetospeed( &options, B9600 );

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
	// UDPServer server("127.0.0.1", 7777);
	UDPServer server("192.168.1.1", 7777);

	printf("Wait for client to be ready\n");
	// wait for client to connect
	//while(server.recv(msg, 10) == -1);
	
	while (1) {
		// Read from the port
		//usleep(10000); // TODO : check response time (maybe with baud rate) and read() returned value is 0 for EOF

		serialport_read_until(fd, buffer, 'x');
		//bufptr = strcpy(bufptr, buffer);
		printf("recu : %s\n", buffer);
		//bsent = server.send(buffer, 3);
		
	}

	close(fd);
}
