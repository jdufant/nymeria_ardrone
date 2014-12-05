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
#define NB_VAL 10

int traitTab(int* tab, int size);

int serialport_read_until(int fd, char* buf, char until)
{
    char b[1];
    int i=0; int n=0; int chread=0;
    
    //printf("read_until\n");
    do { 
        n = read(fd, b, 1);  // read a char at a time
        if( n==-1) return -1;    // couldn't read
        if( n==0 ) {
	  //usleep(5* 1000 ); // wait 1 msec try again
	  continue;
        }
        buf[i] = b[0]; 
	i++;

    } while( b[0] != until);
    
    chread = i-1;

    for (int j = i; i < BUFFER_SIZE; i++)
      {
	buf[i] = '\0';  // null terminate the string
      }

    return chread;
}

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
	    // Chose raw (not processed) output
	    //options.c_oflag &= ~OPOST;
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
  //const char sendBuffer[4];
  int value, tmp_value = 0;

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

	setSerialOptions(options);

	if ( tcsetattr( fd, TCSANOW, &options ) == -1 )
		perror("Error with tcsetattr");
	else
		printf("tcsetattr succeed\nPERROQUET\n");

	// Open UDP server //
	UDPServer server("192.168.1.1", 7777);

	printf("Wait for client to be ready\n");
	// wait for client to connect
	while(server.recv(readBuffer, BUFFER_SIZE) == -1);
	
	while (1) {

	  // Read from the port
	  // TODO : check response time (maybe with baud rate) and read() returned value is 0 for EOF

	  chread = serialport_read_until(fd, readBuffer, 'x');

	  /*msg[3] = '\0';
	  msg[2] = buffer[2];
	  msg[1] = buffer[1];
	  msg[0] = buffer[0];
	  
	  value = atoi(msg);*/
	  
	  if (chread > 0) {
	    printf("recu : %s\n", readBuffer);

	    if((tabVal[cpt_boucle] = atoi(readBuffer)) < 400){
	      cpt_tab ++;
	    }

	    cpt_boucle++;
	  }

	  if (cpt_boucle >=10){
	    value = traitTab(tabVal, cpt_tab);
	    ss << value;
	    tmpStr = ss.str();
	    const char* sendBuffer = tmpStr.c_str();
	    bsent = server.send(sendBuffer, 4);
	    cpt_boucle = 0;
	  }
		
	}

	close(fd);
}

int traitTab(int* tab, int size)
{
  int total = 0;
  int moyenne = 0;
  const int ecart = 20;
  
  for(int i =0; i < size; i++)
    total += tab[i];

  moyenne = total/size;

  for(int i =0; i < size; i++){
    if (abs(tab[i] - moyenne) >= 20)
      for (int j = i; j < size-1; j++)
	tab[j] = tab[j+1];
    size--;
  }
  
  total = 0;
  for(int i =0; i < size; i++)
    total += tab[i];

  return total/size;
}
