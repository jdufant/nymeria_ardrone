#include <nymeria_ardrone/SensorInterface.h>
#include <nymeria_ardrone/NymeriaCheckObstacle.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>

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

	while(ros::ok()){
	
	// 	// TODO receive


		nco.inputCurFrontDist(10.0);
	 	loop_rate.sleep();
	}
	return;
}