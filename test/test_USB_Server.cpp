#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include "UDPWrapper.h"

int main()
{
    printf("Hello world\n");

  int fd, sockfd; /* File descriptor for the port */

    int n;
    int bytes, bsent;
    int i = 0;
 
    char c;
    char msg[100];

    char buffer[10];
    char *bufptr;

    struct termios options;

    fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY | O_NDELAY);

    if (fd == -1) {
        perror("open_port: Unable to open /dev/ttyUSB0 - ");
    }
    else {
        fcntl(fd, F_SETFL, 0);
    }

  tcgetattr( fd, &options );

  /* SEt Baud Rate */

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
    printf ("Error with tcsetattr = %s\n", strerror ( errno ) );
  else
    printf ( "%s\n", "tcsetattr succeed" );
	

 		 	// Open UDP server //

	UDPServer leServer("192.168.1.1", 7777);
	printf("connected\n");
	leServer.recv(msg, 100);

	printf("send : return = %d\n", leServer.send("test",4));

	for (i=0; i<10; i++)
{
buffer[i] = 0;
}
	i = 0;
	while (i < 500) {
    // Read from the port
	usleep(100000);

    bytes = read(fd, buffer, sizeof(buffer));
    printf("number of bytes read is %d\n", bytes);
	
  printf("%d-", buffer[0]);
	printf("%d-", buffer[1]);
	printf("%d-", buffer[2]);
	printf("%d-", buffer[3]);
	printf("%d-", buffer[4]);
	printf("%d-", buffer[5]);
	printf("%d-", buffer[6]);
	printf("%d-", buffer[7]);
	printf("%d-", buffer[8]);
	printf("%d\n", buffer[9]);

    //perror ("read error:");
	i++;
	bsent = leServer.send(buffer, 10);
	}
    close(fd);
return 0;
;
}
