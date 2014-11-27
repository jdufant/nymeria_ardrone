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

void SensorInterface::loop(ros::NodeHandle * n){
	int rate = 50;
	int value =0;
	ros::Rate loop_rate(rate);

	// 50 cm security distance
	NymeriaCheckObstacle nco(n, 80);

	char message[10];
	int nb_char;

	Nymeria nym (n);

	// UDPClient client("127.0.0.1", 7777);
	UDPClient client("192.168.1.1", 7777);
	
	// start communication with server
	while(client.send("ready", 10) <= 0);

	while(ros::ok()){
	
		nb_char = client.recv(message, 3);
		
		if(nb_char > 0)
			printf("Recu : %s__", message);

		printf("\n");
		//value = message[3] || message[2] << 8 || message[1] << 16 || message[0] << 24;
		value = atoi(message);
		// TODO : filter data (sometimes dist is negative)
		if(value > 0)
			nco.inputCurFrontDist(value);

		// hand over commands to Nymeria
		nym.validateStates();

	 	loop_rate.sleep();
	}
	return;
}