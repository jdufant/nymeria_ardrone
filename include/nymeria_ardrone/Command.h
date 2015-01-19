#ifndef NYMERIA_COMMAND_H
#define NYMERIA_COMMAND_H

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/Nymeria.h>

#define KEYCODE_ENTER 0x0A
#define KEYCODE_SPACEBAR 0x20
#define KEYCODE_UP 0x2B
#define KEYCODE_DOWN 0x2D
#define KEYCODE_a 0x61
#define KEYCODE_d 0x64
#define KEYCODE_i 0x69
#define KEYCODE_j 0x6A
#define KEYCODE_k 0x6B
#define KEYCODE_l 0x6C
#define KEYCODE_m 0x6D
#define KEYCODE_o 0x6F
#define KEYCODE_p 0x70
#define KEYCODE_q 0x71
#define KEYCODE_s 0x73
#define KEYCODE_z 0x7A
#define KEYCODE_u 0x75


class TeleopKeyboard
{
	public:
		TeleopKeyboard();
		void keyLoop(ros::NodeHandle * n);  
		ros::NodeHandle * getNH();

	private:
		ros::NodeHandle nh;
		ros::Publisher pub_cmd_move;
		ros::Publisher pub_cmd_takeoff;  
		ros::Publisher pub_cmd_land; 
		geometry_msgs::Twist move_cmd;
};

#endif