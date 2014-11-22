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
	int rate = 10;
	ros::Rate loop_rate(rate);

	NymeriaCheckObstacle nco(n, 20.0);

	char message[4];
	int nb_char;

	UDPClient client("127.0.0.1", 7777);
	// UDPClient client("192.168.1.1", 7777);
	
	while(client.send("go", 2) <= 0);

	while(ros::ok()){
	
	// 	// TODO receive
		nb_char = client.recv(message, 0);
		/*if(message[0] > 120){
			message[0] = 128;
		}*/		
		if(nb_char > 0)
			printf("recu : %d\n", atoi(message));

		nco.inputCurFrontDist(atoi(message));
	 	loop_rate.sleep();
	}
	return;
}