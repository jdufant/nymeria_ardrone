/**Includes**/
#include "/opt/ros/indigo/include/ros/ros.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "ardrone_autonomy/Navdata.h"
#include <signal.h>
#include <termios.h>
#include <stdio.h>

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

void callback (const ardrone_autonomy::Navdata& data){}

static class Nymeria
{
	public:
		Nymeria();
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
		checkStates(int cmd);
		modifyState(int cmd);
		int safeActions[17] ;
};

Nymeria::Nymeria():
	nh.setParam("mutexStateDrone", true);
	nh.setParam("mutexStateObstacle", true);
	nh.setParam("stateDrone", 0);
	nh.setParam("stateObstacle", false);
	{
		//nh.param("scale_angular", a_scale_, a_scale_);
		nh.param("scale_linear", l_scale_, l_scale_);
	}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_keyboard");
	TeleopKeyboard teleop;

	signal(SIGINT,quit);

	teleop.keyLoop();

	return(0);
}


void TeleopKeyboard::keyLoop()
{
	char c;
	bool dirty=false;
	std_msgs::Empty empty_cmd;
	geometry_msgs::Twist move_cmd;

		ros::Publisher pub_cmd_move = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
		ros::Publisher pub_cmd_takeoff = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 10);
		ros::Publisher pub_cmd_land = nh.advertise<std_msgs::Empty>("ardrone/land", 10);

	// get the console in raw mode                                                              
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &=~ (ICANON | ECHO);
	// Setting a new line, then end of file                        
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from the keyboard");
	puts("---------------------------");

	puts("x : stay stationary");
	puts("");
	puts("z/d : move forward/backward");
	puts("q/d : rotate left/right");
	puts("o/l : move up/down");
	puts("");

	puts("t/g : takeoff/land");
	puts("");

	puts("y/h : increase/decrease max speeds by 10");
	puts("u/j : increase/decrease only linear speed by 10%");
	puts("i/k : increase/decrease only angular speed by 10 percent");
	puts("");

	puts("anything else : stop");
	puts("CTRL-C to quit");
	puts("---------------------------");

	int count=0;

	while(ros::ok())
	{
		count++;
		// get the next event from the keyboard  
		if(read(kfd, &c, 1) < 0){
			perror("read():");
			exit(-1);
		}

		linear_=angular_=0;
		ROS_DEBUG("value: 0x%02X\n", c);
		std::stringstream ss;

		switch(c){
			case KEYCODE_L:
				ROS_WARN("LEFT");
				angular_ = 1.0;
				dirty = true;
				break;
			case KEYCODE_Q:
				ROS_WARN("STOP");
				angular_ = 0.0;
				linear_ = 0.0;
				dirty = true;
				break;
		}

		geometry_msgs::Twist twist;
		
		if(dirty == true){
			pub_cmd_move.publish(twist);    
			dirty=false;
		}
	}


	return;
}
