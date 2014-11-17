/**Includes**/
#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>

/** My program **/
/* No test have been done so I am not sure it is working, especially the "while (state == x)" 
   This program is supposed to control the drone by sending commands like "takeoff", "landing" and "move forward"
   Joffrey, it would be great if you could modify your code for the keyboard so that the 2 nodes can work together
   We should make a ROS class for everyone. You can do it on Wednesday afternoon if you wish Joffrey :)
   Otherwise you can wait for me and we'll do one next week
   Commande.h file is useless ^^
   cheers ! */


/**Global variables where important data are stored **/
/* (I don't like to use them but this is an easy temporary solution, we should use classes and arguments in the futur)*/

ardrone_autonomy::Navdata dNavdata;
/*dNavdata stores information about the drone (see https://github.com/AutonomyLab/ardrone_autonomy at "Legacy Navigation data" for more info)
I only use the "state" variable here*/

int cmd;
/*cmd stores the command you want to send to the drone. For now, only take off(1), landing(2), and moving forward(3) is implemented  */


/**Callback functions called in the subscribers**/
/* (same as above, use of classes and methods are better I think...*/
void stateCallback (const ardrone_autonomy::Navdata& data)
{
  dNavdata.state = data.state;
}

void cmdCallback (const std_msgs::UInt8ConstPtr& cmd_r)
{
  cmd = cmd_r->data; 
  /* it's a bit tricky here, I struggled figuring out what was wrong...
     the type of cmd_r is an UInt8ConstPtr which is a boost::shared_ptr class/structure/type (pick one, I don't really know myself...) 
     in big letters you need to put the "->data" to get the value (like for an argument of a class pointer)*/
}

/**MAIN**/
int main(int argc, char **argv)
{
  /**variables declaration for the messages to be sent on the topics**/
  std_msgs::Empty empty_msg; //message for take off and landing
  geometry_msgs::Twist move_msg;//message for moving

  
  ros::init(argc, argv, "commande"); //init and name the node ("commande" here)
  ros::spinOnce(); //not sure what this does but it looked compulsory
  
  ros::NodeHandle n; //node handler

  /**Publishers, to send messages on topics**/
  ros::Publisher pub_cmd_takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1000);
  ros::Publisher pub_cmd_land = n.advertise<std_msgs::Empty>("ardrone/land", 1000);
  ros::Publisher pub_cmd_move = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  /**Subscribers, to read messages from topics**/
  /*here the callback functions are called in order to update the cmd and dNavdata variables*/
  ros::Subscriber sub_cmd = n.subscribe("drone_cmd",7,cmdCallback);
  ros::Subscriber sub_navdata = n.subscribe("ardrone/navdata",128,stateCallback);

  /*Initialize loop rate*/
  ros::Rate loop_rate(10); //10Hz

  /*Waiting for a state = 1 (meaning the drone is initialized)*/
  while (ros::ok() && dNavdata.state != 1)
    {
      loop_rate.sleep();
    }

  /*Infinite loop until the node is stopped*/
  while (ros::ok())
    {
      /*Execute the different action depending on the cmd variable*/
      switch (cmd)
	{
	case 1:
	  //Takeoff
	  pub_cmd_takeoff.publish(empty_msg);
	  ROS_INFO("Decollage...\n");
	  //here we wait while the drone is taking off
	  while(dNavdata.state == 6);
	  break;
	
	case 3:
	  //Move forward
	  move_msg.linear.x = 2.0; //Changing the message to make the drone move
	  pub_cmd_move.publish(move_msg); //Publishing msg
	  ROS_INFO("Deplacement :\nlinear.x = %f\nlinear.y = %f\nlinear.z = %f\nangular.x = %f\nangular.y = %f\nangular.z = %f\n", 
		   move_msg.linear.x, move_msg.linear.y, move_msg.linear.z, 
		   move_msg.angular.x,move_msg.angular.y,move_msg.angular.z);
	  ros::Duration(1.0).sleep(); //I wrote a sleep here because I wasnt sure which state value to write in the while loop. But we should write same as above : while (state == ?);
	
	  move_msg.linear.x = 0.0; //Changing the message to make the drone stop

	  //Stop
	  pub_cmd_move.publish(move_msg);
	  ROS_INFO("Deplacement :\nlinear.x = %f\nlinear.y = %f\nlinear.z = %f\nangular.x = %f\nangular.y = %f\nangular.z = %f\n", 
		   move_msg.linear.x, move_msg.linear.y, move_msg.linear.z, 
		   move_msg.angular.x,move_msg.angular.y,move_msg.angular.z);
	  ros::Duration(1.0).sleep();//Same
	  break;

	case 2:
	  //Landing
	  pub_cmd_land.publish(empty_msg);
	  ROS_INFO("Atterrissage...\n");
	  //waiting while the drone is landing
	  while(dNavdata.state == 8);
	  break;

	default:
	  ROS_INFO("commande inconnue\n");
	  break;
	}

      loop_rate.sleep(); //relative to the loop rate

    }


  return 0;
}
