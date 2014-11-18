#ifndef SRC_HEADERS_POLLING_H
#define SRC_HEADERS_POLLING_H

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/Nymeria.h>

class Polling
{
	public:
		Polling();

	private:
		ros::NodeHandle nh;
		ros::Publisher pub_cmd_move;
		ros::Publisher pub_cmd_takeoff;  
		ros::Publisher pub_cmd_land; 
		geometry_msgs::Twist move_cmd;
};

#endif