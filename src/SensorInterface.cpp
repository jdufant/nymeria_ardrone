#include <nymeria_ardrone/SensorInterface.h>
#include <nymeria_ardrone/NymeriaCheckObstacle.h>
#include <nymeria_ardrone/UDPWrapper.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/Nymeria.h>

#define BUFFER_SIZE 4

ros::NodeHandle * SensorInterface::getNH(){
	return &nh;
}

SensorInterface::SensorInterface(){}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensor_interface");
	
	SensorInterface si;

	si.loop(si.getNH());

	return 0;
}

void SensorInterface::cutBuffer(char* bufferIn, int size, int& valeur) {
  
	int i=0;
	char tmp_buffer[BUFFER_SIZE];
	while (i < size){
		if (bufferIn[i] == 'x')
			break;
		else
			tmp_buffer[i] = bufferIn[i];
		i++;
	}

	valeur = atoi(tmp_buffer);
}

void SensorInterface::loop(ros::NodeHandle * n){
	int rate = 50;
	int value = 0;
	int cutValue = 0;
	ros::Rate loop_rate(rate);

	NymeriaCheckObstacle nco(n);

	char message[BUFFER_SIZE];
	int nb_char;

	UDPClient client("192.168.1.1", 7777);
	
	// start communication with server
	while(client.send("go", BUFFER_SIZE) <= 0);

	while(ros::ok()){
	
		nb_char = client.recv(message, BUFFER_SIZE);
		if (nb_char != 0)
		 	cutBuffer(message, nb_char, cutValue);
		
		if(nb_char > 0)
		  printf("Recu %d char : %d__", nb_char, cutValue);

		printf("\n");
		
		if (cutValue >=0 && cutValue < nco.getSensorMaxRange())
		  nco.inputCurFrontDist(cutValue);

	 	loop_rate.sleep();
	}
	return;
}
