#ifndef NYMERIA_ENVIRONMENT
#define NYMERIA_ENVIRONMENT

/**Includes**/
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "ardrone_autonomy/Navdata.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define CHECK		0
#define	M_FORWARD 	1
#define	M_BACKWARD	2
#define	M_LEFT		3
#define	M_RIGHT		4
#define	M_UP		5
#define	M_DOWN		6
#define	T_LEFT		7
#define	T_RIGHT		8
		
#define	STOP		9
#define	TAKEOFF		10
#define	LAND		11
#define	E_STOP		12

#define	I_M_SPEED	13	
#define	D_M_SPEED	14	
#define	I_L_SPEED	15	
#define	D_L_SPEED	16	
#define	I_A_SPEED	17	
#define	D_A_SPEED	18

#define NO_OBSTACLE	20
#define O_FRONT		21


ardrone_autonomy::Navdata navData;
void stateDroneCallback (const ardrone_autonomy::Navdata& data);

class Nymeria
{
	public:
		Nymeria();
		Nymeria(ros::NodeHandle n);
		void moveForward();
		void moveBackward();
		void moveLeft();
		void moveRight();
		void moveUp();
		void moveDown();
		void turnLeft();
		void turnRight();
		void stop();
		void takeOff();
		void land();
		void emergencyStop();
		void increaseMaxSpeed();
		void decreaseMaxSpeed();
		void increaseLinearSpeed();
		void decreaseLinearSpeed();
		void increaseAngularSpeed();
		void decreaseAngularSpeed();
	private:
		ros::NodeHandle nh;
		  /**Publishers, to send messages on topics**/
		ros::Publisher pub_cmd_takeoff;
		ros::Publisher pub_cmd_land;
		ros::Publisher pub_cmd_move;
		ros::Publisher pub_cmd_reset;
		
		ros::Subscriber sub_navdata;
		
		std_msgs::Empty empty_msg;
		geometry_msgs::Twist move_msg;
		int safeActions[17];
		bool isSafeAction(int cmd);
		void validateStates(int cmd);
		void modifyStateDrone(int cmd);
		void nymeriaRoutine(int cmd);
		void triggerAction(int cmd);
		void reactionRoutine();

};

Nymeria::Nymeria(){};
Nymeria::Nymeria(ros::NodeHandle n){
	move_msg.linear.x = 0;
	move_msg.linear.y = 0;
	move_msg.linear.z = 0;
	move_msg.angular.x = 0;
	move_msg.angular.y = 0;
	move_msg.angular.z = 0;

	/* Initialize safeActions. */
	safeActions[0] = M_BACKWARD;
	safeActions[1] = M_LEFT;
	safeActions[2] = M_RIGHT;
	safeActions[3] = M_UP;
	safeActions[4] = M_DOWN;
	safeActions[5] = T_LEFT;
	safeActions[6] = T_RIGHT;
	safeActions[7] = STOP;
	safeActions[8] = TAKEOFF;
	safeActions[9] = LAND;
	safeActions[10] = E_STOP;
	safeActions[11] = I_M_SPEED;
	safeActions[12] = D_M_SPEED;
	safeActions[13] = I_L_SPEED;
	safeActions[14] = D_L_SPEED;
	safeActions[15] = I_A_SPEED;
	safeActions[16] = D_A_SPEED;

	nh = n;

	/* Set parameters shared with all ROS nodes. */ 
	nh.setParam("mutexStateDrone", true);
	nh.setParam("mutexStateObstacle", true);
	nh.setParam("stateDrone", 0);
	nh.setParam("stateObstacle", false);

	pub_cmd_takeoff = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
	pub_cmd_land = nh.advertise<std_msgs::Empty>("ardrone/land", 10);
	pub_cmd_reset = nh.advertise<std_msgs::Empty>("ardrone/reset", 10);
	pub_cmd_move = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	sub_navdata = nh.subscribe("ardrone/navdata", 10, stateDroneCallback);

};

#endif
