#ifndef NYMERIA_ENVIRONMENT_H
#define NYMERIA_ENVIRONMENT_H

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/NymeriaConstants.h>
#include <nymeria_ardrone/NymeriaMutexDrone.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>

/**
 * Declaration of the class Nymeria, that declares all functionalities
 * in order to allow for drone navigation with obstacle detection and avoidance.
 */
class Nymeria
{
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
		void increaseMaxSpeed();
		void decreaseMaxSpeed();
		void increaseLinearSpeed();
		void decreaseLinearSpeed();
		void increaseAngularSpeed();
		void decreaseAngularSpeed();
		

	private:
		ros::NodeHandle * nh;
		  /**Publishers, to send messages on topics**/
		ros::Publisher pub_cmd_takeoff;
		ros::Publisher pub_cmd_land;
		ros::Publisher pub_cmd_move;
		ros::Publisher pub_cmd_reset;
		ardrone_autonomy::Navdata navData;		
		ros::Subscriber sub_navdata;
		
		std_msgs::Empty empty_msg;
		geometry_msgs::Twist move_msg;
		
		int stateDrone;
		int stateObstacle;

		float speed;

		int safeActions[17];
		bool isSafeAction(int cmd);
		void validateStates(int cmd);
		void modifyStateDrone(int cmd);
		void nymeriaRoutine(int cmd);
		void triggerAction(int cmd);
		void reactionRoutine();
		void stateDroneCallback(const ardrone_autonomy::Navdata& data);

};

#endif
