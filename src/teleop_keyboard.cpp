/**Includes**/
//#include "teleop_keyboard.h"
//#include "/opt/ros/indigo/include/ros/ros.h"
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "ardrone_autonomy/Navdata.h"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_q 0x71
#define KEYCODE_d 0x64
#define KEYCODE_z 0x7A
#define KEYCODE_s 0x73
#define KEYCODE_SPACEBAR 0x20
#define KEYCODE_ENTER 0x0A
#define KEYCODE_UP 0x2B
#define KEYCODE_DOWN 0x2D
#define KEYCODE_a 0x61

void callback (const ardrone_autonomy::Navdata& data){}

class TeleopKeyboard
{
public:
  TeleopKeyboard();
  void keyLoop();

private:
  ros::NodeHandle nh;  
  ros::Publisher pub_cmd_move;
  ros::Publisher pub_cmd_takeoff;  
  ros::Publisher pub_cmd_land; 
  geometry_msgs::Twist move_cmd;
};

TeleopKeyboard::TeleopKeyboard()
{
  move_cmd.linear.x =0;
  move_cmd.linear.y =0;
  move_cmd.linear.z =0;
  move_cmd.angular.x =0;
  move_cmd.angular.y =0;
  move_cmd.angular.z =0;
  
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
	bool dirty(false), flying(false), hovering(true);
	std_msgs::Empty empty_msg;

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

	puts("spacebar : hovering mode");
	puts("");
	puts("z/s : move forward/backward");
	puts("q/d : rotate left/right");
	puts("<up key>/<down key> : move up/down");
	puts("");

	puts("enter : takeoff/landing");
	puts("");

	/*puts("y/h : increase/decrease max speeds by 10");
	puts("u/j : increase/decrease only linear speed by 10%");
	puts("i/k : increase/decrease only angular speed by 10 percent");
	puts("");*/

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

		ROS_INFO("value: 0x%02X\n", c);
		//std::stringstream ss;

		switch(c){
			case KEYCODE_q:
			  ROS_WARN("LEFT");
			  move_cmd.angular.z = 0.2;
			  dirty = true;
			  break;	
		
		        case KEYCODE_d:
			  ROS_WARN("RIGHT");
			  move_cmd.angular.z = -0.2;
			  dirty = true;
			  break;

		        case KEYCODE_z:
			  ROS_WARN("FORWARD");
			  move_cmd.linear.x = 0.1;
			  dirty = true;
			  break;
	         	
		       case KEYCODE_s:
			  ROS_WARN("BACKWARD");
			  move_cmd.linear.x = -0.1;
			  dirty = true;
			  break;
		
		        case KEYCODE_UP:
			  ROS_WARN("UP");
			  move_cmd.linear.z = 0.5;
			  dirty = true;
			  break;

			case KEYCODE_DOWN:
			  ROS_WARN("DOWN");
			  move_cmd.linear.z = -0.5;
			  dirty = true;
			  break;

			case KEYCODE_SPACEBAR:
			  ROS_WARN("STOP");
			  move_cmd.linear.x = 0.0;
			  move_cmd.angular.x = 0.0;
			  dirty = true;
			  break;

			case KEYCODE_ENTER:
			  if (!flying) {
			    ROS_WARN("TAKEOFF");
			    pub_cmd_takeoff.publish(empty_msg);
			    flying = true;
			  }
			  else {
			    ROS_WARN("LAND");
			    pub_cmd_land.publish(empty_msg);
			    flying = false;
			  }
				break;				

		        case KEYCODE_a:
			  if(hovering == false)
			    {
			      ROS_WARN("Hovering mode ON");
			      move_cmd.angular.x = 0.0;
			      hovering = true;
			    }

			  else
			    {
			      ROS_WARN("Hovering mode OFF");
			      move_cmd.angular.x = 1.0;
			      hovering = false;
			    }
			  break;
		}
		
		if(dirty == true){
			pub_cmd_move.publish(move_cmd);    
			dirty=false;
			if (move_cmd.angular.z != 0.0)
			  {
			    ros::Duration(0.50).sleep();
			    move_cmd.angular.z = 0.0;
			    pub_cmd_move.publish(move_cmd); 
			  }
			if (move_cmd.linear.z != 0.0)
			  {
			    ros::Duration(0.50).sleep();
			    move_cmd.linear.z = 0.0;
			    pub_cmd_move.publish(move_cmd);
			  }
		}
	}


	return;
}
