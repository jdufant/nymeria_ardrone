/**Includes**/
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "ardrone_autonomy/Navdata.h"

#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Publisher cmd_pub_;  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  cmd_pub_ = nh_.advertise<std_msgs::String>("raupe/cmd", 1000);
  vel_pub_ = nh_.advertise<turtlesim::Velocity>("raupe/vel", 1);
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
  ros::init(argc, argv, "teleop");
  TeleopTurtle teleop;

  signal(SIGINT,quit);

  teleop.keyLoop();
 
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;
  std_msgs::String cmd;
 
  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                        
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
  int count=0;

  for(;;)
  {
        count++;
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
        std::stringstream ss;
 
    switch(c)
    {
      case KEYCODE_L:
        ROS_WARN("LEFT");
        angular_ = 1.0;
        dirty = true;
                ss << "key LEFT\n\r";
                cmd.data = ss.str();
        break;
      case KEYCODE_R:
        ROS_WARN("RIGHT");
        angular_ = -1.0;
        dirty = true;
                ss << "key RIGHT\n\r";
                cmd.data = ss.str();
        break;
      case KEYCODE_U:
        ROS_WARN("UP");
        linear_ = 1.0;
        dirty = true;
                ss << "key UP\n\r";
                cmd.data = ss.str();
        break;
      case KEYCODE_D:
        ROS_WARN("DOWN");
        linear_ = -1.0;
        dirty = true;
                ss << "key DOWN\n\r";
                cmd.data = ss.str();
        break;
      case KEYCODE_Q:
        ROS_WARN("STOP");
        angular_ = 0.0;
        linear_ = 0.0;
        dirty = true;
                ss << "key STOP\n\r";
                cmd.data = ss.str();
        break;
    }
   
    turtlesim::Velocity vel;
    vel.angular = a_scale_*angular_;
    vel.linear = l_scale_*linear_;
    if(dirty ==true)
    {
      vel_pub_.publish(vel);    
      cmd_pub_.publish(cmd);    
      dirty=false;
    }
  }


  return;
}
