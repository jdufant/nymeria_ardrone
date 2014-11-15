#include "Nymeria.h"

/* PRIVATE methods */
/**
* Helper function in order to forward user's command
* to drone.
* Sets drone's navigation state in transmitted message
*
* @param data - navigation state.
*/
// void stateDroneCallback (const ardrone_autonomy::Navdata& data){
// 	navData.state = data.state;
// }


Nymeria::Nymeria(){};
Nymeria::Nymeria(ros::NodeHandle * n)
	{

	this->cst = NymeriaConstants();

	speed = 0.2;

	move_msg.linear.x = 0;
	move_msg.linear.y = 0;
	move_msg.linear.z = 0;
	move_msg.angular.x = 0;
	move_msg.angular.y = 0;
	move_msg.angular.z = 0;

	/* Initialize safeActions. */
	safeActions[0] = cst.M_BACKWARD;
	safeActions[1] = cst.M_LEFT;
	safeActions[2] = cst.M_RIGHT;
	safeActions[3] = cst.M_UP;
	safeActions[4] = cst.M_DOWN;
	safeActions[5] = cst.T_LEFT;
	safeActions[6] = cst.T_RIGHT;
	safeActions[7] = cst.STOP;
	safeActions[8] = cst.TAKEOFF;
	safeActions[9] = cst.LAND;
	safeActions[10] = cst.E_STOP;
	safeActions[11] = cst.I_M_SPEED;
	safeActions[12] = cst.D_M_SPEED;
	safeActions[13] = cst.I_L_SPEED;
	safeActions[14] = cst.D_L_SPEED;
	safeActions[15] = cst.I_A_SPEED;
	safeActions[16] = cst.D_A_SPEED;

	nh = n;

	/* Set parameters shared with all ROS nodes. */ 
	// nh->setParam("nymeria/mutexStateDrone", true);
	// nh->setParam("nymeria/mutexStateObstacle", true);
	nh->setParam("nymeria/stateDrone", 0);
	nh->setParam("nymeria/stateObstacle", 0);

	pub_cmd_takeoff = nh->advertise<std_msgs::Empty>("ardrone/takeoff", 10);
	pub_cmd_land = nh->advertise<std_msgs::Empty>("ardrone/land", 10);
	pub_cmd_reset = nh->advertise<std_msgs::Empty>("ardrone/reset", 10);
	pub_cmd_move = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
	
	//sub_navdata = nh->subscribe("ardrone/navdata", 10, stateDroneCallback);

};


/**
 * Helper function in order to forward user's command
 * to drone.
 * Sets drone's navigation state in transmitted message
 *
 * @param data - navigation state.
 */
//void stateDroneCallback (const ardrone_autonomy::Navdata& data){
//	navData.state = data.state;
//}

/**
 * Procedure that processes incoming command.
 * @param cmd - incoming command.
 */
void Nymeria::nymeriaRoutine(int cmd){
	if(isSafeAction(cmd))
		triggerAction(cmd);
	else
		validateStates(cmd);
}

/**
 * Forward command to drone.
 * @param cmd - incoming command.
 */
void Nymeria::triggerAction(int cmd){
	switch(cmd){
		case (&cst)->M_FORWARD:
			ROS_INFO("M_FORWARD");
			move_msg.linear.x = 1;
			break;
		case cst.M_BACKWARD:
			ROS_INFO("M_BACKWARD");
			move_msg.linear.x = -1;
			break;
		case cst.M_LEFT:
			ROS_INFO("M_LEFT");
			move_msg.linear.y = 1;
			break;
		case cst.M_RIGHT:
			ROS_INFO("M_RIGHT");
			move_msg.linear.y = -1;
			break;
		case cst.M_UP:
			ROS_INFO("M_UP");
			move_msg.linear.z = 1;
			break;
		case cst.M_DOWN:
			ROS_INFO("M_DOWN");
			move_msg.linear.z = -1;
			break;
		case cst.T_LEFT:
			ROS_INFO("T_LEFT");
			move_msg.angular.z = 1;
			break;
		case cst.T_RIGHT:
			ROS_INFO("T_RIGHT");
			move_msg.angular.z = -1;
			break;

		case cst.STOP:
			ROS_INFO("STOP");
			move_msg.linear.x = 0;
			move_msg.linear.y = 0;
			move_msg.linear.z = 0;
			move_msg.angular.x = 0;
			move_msg.angular.y = 0;
			move_msg.angular.z = 0;
			break;

		case cst.TAKEOFF:
			ROS_INFO("TAKEOFF");
			pub_cmd_takeoff.publish(empty_msg);
			while(navData.state == 6);
			break;
		case cst.LAND:
			ROS_INFO("LAND");
			pub_cmd_land.publish(empty_msg);
			while(navData.state == 8);
			break;
		case cst.E_STOP:
			ROS_INFO("E_STOP");
			pub_cmd_reset.publish(empty_msg);
			break;

		case cst.I_M_SPEED:
			speed += 0.1;
			break;
		case cst.D_M_SPEED:
			speed -= 0.1;
			break;
		case cst.I_L_SPEED:
			speed += 0.1;
			break;
		case cst.D_L_SPEED:
			speed -= 0.1;
			break;
		case cst.I_A_SPEED:
			speed += 0.1;
			break;
		case cst.D_A_SPEED:
			speed -= 0.1;
			break;

		default:
			ROS_WARN("Command unknown\n");
			break;
	}

	move_msg.linear.x *= speed;
	move_msg.linear.y *= speed;
	move_msg.linear.z *= speed;
	move_msg.angular.x *= speed;
	move_msg.angular.y *= speed;
	move_msg.angular.z *= speed;

	pub_cmd_move.publish(move_msg);
	ROS_INFO("Move :\nlinear.x = %f\nlinear.y = %f\nlinear.z = %f\nangular.x = %f\nangular.y = %f\nangular.z = %f\n", 
	move_msg.linear.x, move_msg.linear.y, move_msg.linear.z, 
	move_msg.angular.x,move_msg.angular.y,move_msg.angular.z);

	// reinitialise move_msg
	move_msg.linear.x = 0;
	move_msg.linear.y = 0;
	move_msg.linear.z = 0;
	move_msg.angular.x = 0;
	move_msg.angular.y = 0;
	move_msg.angular.z = 0;

	modifyStateDrone(cmd);
}



void Nymeria::modifyStateDrone(int cmd){
	// beetween 1 and 12
	if(cmd >= 1 || cmd <= 12){
		// TODO MUTEX parameter 1
		if(nh->hasParam("nymeria_ardrone/stateDrone"))
			nh->setParam("nymeria_ardrone/stateDrone", cmd);
	}

	validateStates(cst.CHECK);
}
/**
* Check whether current state of drone and current state of
* obstacle are compatible with user's command.
*
* Recursive function, that terminates, when either there is
* a conflict between states and input command (trigger
* reactionRoutine) or
* states and command are compatible and the command has been
* sent the first time.
*
* @param cmd - incoming command: Either M_FORWARD (termination)
* or CHECK (recursive function call to validateStates(CHECK).
*/
void Nymeria::validateStates(int cmd){
	int tmpStateDrone;
	int tmpStateObstacle;
	// are the two states in conflict
	if(nh->getParam("nymeria_ardrone/stateDrone", stateDrone))
		tmpStateDrone = stateDrone;
	// TODO exception if false + function

	if(nh->getParam("nymeria_ardrone/stateObstacle", stateObstacle))
		tmpStateObstacle = stateObstacle;

	// TODO exception if false


	if((tmpStateDrone == cst.M_FORWARD) && (tmpStateObstacle == cst.O_FRONT)){
		reactionRoutine();
		// exit recursion/loop
		return;
	}
	else if (cmd > 0){
		triggerAction(cmd);
		// exit recursion/loop
		return;
	}

	validateStates(cst.CHECK);
	return;
}

/**
 * Does the user's command belong to the list of safeActions,
 * i.e. can the command be safely forwarded to the drone?
 *
 * @param cmd - incoming command.
 * @return true: yes, command can be forwarded.
 * 		   false: no, check for obstacles is necessary.
 */
bool Nymeria::isSafeAction(int cmd){
	for (int i = 0; i < sizeof(safeActions)/sizeof(*safeActions); i++){
		if (safeActions[i] == cmd){
			return true;
		}
	}
	return false;
}
/** TODO
* Routine in order to make drone stop in front of obstacle and
* then bypass it, i.e. navigate around it.
*/
void Nymeria::reactionRoutine(){
	triggerAction(cst.STOP);
}

/* PUBLIC methods */

/**
 * Command in order to move drone forward.
 */
void Nymeria::moveForward(){
	nymeriaRoutine(cst.M_FORWARD);
}
/**
* Command in order to move drone backward.
*/
void Nymeria::moveBackward(){
	nymeriaRoutine(cst.M_BACKWARD);
}
/**
* Command in order to make drone rotate to the left.
*/
void Nymeria::moveLeft(){
	nymeriaRoutine(cst.M_LEFT);
}

/**
* Command in order to make drone rotate to the right.
*/

void Nymeria::moveRight(){
	nymeriaRoutine(cst.M_RIGHT);
}
/**
* Command in order to move drone upward,
* i.e. increase altitude.
*/
void Nymeria::moveUp(){
	nymeriaRoutine(cst.M_UP);
}

/**
* Command in order to move drone downward,
* i.e. decrease altitude.
*/
void Nymeria::moveDown(){
	nymeriaRoutine(cst.M_DOWN);
}
/**
* Command in order to move drone to the left.
*/
void Nymeria::turnLeft(){
	nymeriaRoutine(cst.T_LEFT);
}

/**
* Command in order to move drone to the right.
*/
void Nymeria::turnRight(){
	nymeriaRoutine(cst.T_RIGHT);
}

/**
* Command in order to stop the drone's movement,
* i.e. stay at current position.
*/
void Nymeria::stop(){
	nymeriaRoutine(cst.STOP);
}
/**
* Command in order to make the drone take off.
*/
void Nymeria::takeOff(){
	nymeriaRoutine(cst.TAKEOFF);
}
/**
* Command in order to make the drone land,
* i.e. underneath current position.
*/
void Nymeria::land(){
	nymeriaRoutine(cst.LAND);
}

/**
* Command in order to make drone stop and
* immediately land.
*/
void Nymeria::emergencyStop(){
	nymeriaRoutine(cst.E_STOP);
}
/**
* Command in order to increase the maximum
* speed by 10%.
*/
void Nymeria::increaseMaxSpeed(){
	nymeriaRoutine(cst.I_M_SPEED);
}
/**
* Command in order to decrease the maximum
* speed by 10%.
*/
void Nymeria::decreaseMaxSpeed(){
	nymeriaRoutine(cst.D_M_SPEED);
}

/**
* Command in order to increase the linear
* speed by 10%.
*/
void Nymeria::increaseLinearSpeed(){
	nymeriaRoutine(cst.I_L_SPEED);
}

/**
 * Command in order to decrease the linear
 * speed by 10%.
 */

void Nymeria::decreaseLinearSpeed(){
	nymeriaRoutine(cst.D_L_SPEED);
}

/**
* Command in order to increase the angular
* speed by 10%.
*/
void Nymeria::increaseAngularSpeed(){
	nymeriaRoutine(cst.I_A_SPEED);
}

/**
 * Command in order to decrease the angular
 * speed by 10%.
 */
void Nymeria::decreaseAngularSpeed(){
	nymeriaRoutine(cst.D_A_SPEED);
}

