#ifndef NYMERIA_ENVIRONMENT_H
#define NYMERIA_ENVIRONMENT_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/NymeriaConstants.h>
#include <nymeria_ardrone/Controller.h>

/**
 * Definitions of the class Nymeria, that declares all functionalities
 * in order to allow for drone navigation with obstacle detection and avoidance.
 */
class Nymeria
{
	friend class Controller;
	public:
		Nymeria();
		Nymeria(ros::NodeHandle * n);
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
		void increaseMaxLinearSpeed();
		void decreaseMaxLinearSpeed();
		void increaseMaxAngularSpeed();
		void decreaseMaxAngularSpeed();
		void increaseLinearSpeed();
		void decreaseLinearSpeed();
		void increaseAngularSpeed();
		void decreaseAngularSpeed();
		double getSecurityDist();
		void setSecurityDist(double secDist);
		double getMaxLinearSpeed();
		void setMaxLinearSpeed(double speed);
		double getLinearSpeed();
		void setLinearSpeed(double speed);
		double getMaxAngularSpeed();
		void setMaxAngularSpeed(double speed);
		double getAngularSpeed();
		
	private:
		ros::NodeHandle * nh;
		ros::Publisher pub_cmd_takeoff;
		ros::Publisher pub_cmd_land;
		ros::Publisher pub_cmd_move;
		ros::Publisher pub_cmd_reset;
		ardrone_autonomy::Navdata navData;		
		ros::Subscriber sub_navdata;
		
		std_msgs::Empty empty_msg;
		geometry_msgs::Twist move_msg;

		int lastCmd;
		double maxLinearSpeed;
		double maxAngularSpeed;
		double linearSpeed;
		double angularSpeed;
		int safeActions[20];

		/* Initialization helper functions; */
		void init_safeActions();
		void init_rosParams();
		void init_move_msg();
		void init_publishers();
		
		double getParameter(char * str);
		int validateStates();
		bool isSafeAction(int cmd);
		bool obstaclePossible();
		bool underSecurityDist();
		int triggerAction(int cmd, double factor = 1.0);
		void reactionRoutine();
		void keepSecurityDistance();
		void slowDown();
		bool inRange(double min, double max, double value);

};

#endif
