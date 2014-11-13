#include "Nymeria.h"

/* PRIVATE methods */

/**
 * Helper function in order to forward user's command
 * to drone.
 * Sets drone's navigation state in transmitted message
 *
 * @param data - navigation state.
 */
void stateDroneCallback (const ardrone_autonomy::Navdata& data){
	navData.state = data.state;
}

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
		case M_FORWARD:
			ROS_INFO("M_FORWARD");
			move_msg.linear.x = 1;
			break;
		case M_BACKWARD:
			ROS_INFO("M_BACKWARD");
			move_msg.linear.x = -1;
			break;
		case M_LEFT:
			ROS_INFO("M_LEFT");
			move_msg.angular.z = 1;
			break;
		case M_RIGHT:
			ROS_INFO("M_RIGHT");
			move_msg.angular.z = -1;
			break;
		case M_UP:
			ROS_INFO("M_UP");
			move_msg.linear.z = 1;
			break;
		case M_DOWN:
			ROS_INFO("M_DOWN");
			move_msg.linear.z = -1;
			break;
		case T_LEFT:
			ROS_INFO("T_LEFT");
			move_msg.linear.y = 1;
			break;
		case T_RIGHT:
			ROS_INFO("T_RIGHT");
			move_msg.linear.y = 1;
			break;

		case STOP:
			ROS_INFO("STOP");
			move_msg.linear.x = 0;
			move_msg.linear.y = 0;
			move_msg.linear.z = 0;
			move_msg.angular.x = 0;
			move_msg.angular.y = 0;
			move_msg.angular.z = 0;
			break;

		case TAKEOFF:
			ROS_INFO("TAKEOFF");
			pub_cmd_takeoff.publish(empty_msg);
			while(navData.state == 6);
			break;
		case LAND:
			ROS_INFO("LAND");
			pub_cmd_land.publish(empty_msg);
			while(navData.state == 8);
			break;
		case E_STOP:
			ROS_INFO("E_STOP");
			pub_cmd_reset.publish(empty_msg);
			break;

		case I_M_SPEED:break;
		case D_M_SPEED:break;
		case I_L_SPEED:break;
		case D_L_SPEED:break;
		case I_A_SPEED:break;
		case D_A_SPEED:break;

		default:
			ROS_WARN("Command unknown\n");
			break;
	}

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
		// hasParam
		// setParam <- CMD
	}

	validateStates(CHECK);

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
	// are the two states in conflict
	int tmpStateDrone; // = nh.getParambalaba;
	int tmpStateObstacle; //  = nh.getParambalaba;

	// TODO MUTEX parameter 1
	// hasParam
	// setParam <- CMD
	if(tmpStateDrone == M_FORWARD && tmpStateObstacle == O_FRONT){
		reactionRoutine();
		// exit recursion/loop
		return;
	}
	else if (cmd > 0){
		triggerAction(cmd);
		// exit recursion/loop
		return;
	}

	validateStates(CHECK);
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
	for (int i = 0; i < sizeof(safeActions)/sizeof(*safeActions); i++)
	{
		if (safeActions[i] == cmd)
		{
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

}

/* PUBLIC methods */

/**
 * Command in order to move drone forward.
 */
void Nymeria::moveForward(){
	nymeriaRoutine(M_FORWARD);
}

/**
 * Command in order to move drone backward.
 */
void Nymeria::moveBackward(){
	nymeriaRoutine(M_BACKWARD);
}

/**
 * Command in order to make drone rotate to the left.
 */
void Nymeria::moveLeft(){
	nymeriaRoutine(M_LEFT);
}

/**
 * Command in order to make drone rotate to the right.
 */
void Nymeria::moveRight(){
	nymeriaRoutine(M_RIGHT);
}

/**
 * Command in order to move drone upward,
 * i.e. increase altitude.
 */
void Nymeria::moveUp(){
	nymeriaRoutine(M_UP);
}

/**
 * Command in order to move drone downward,
 * i.e. decrease altitude.
 */
void Nymeria::moveDown(){
	nymeriaRoutine(M_DOWN);
}

/**
 * Command in order to move drone to the left.
 */
void Nymeria::turnLeft(){
	nymeriaRoutine(T_LEFT);
}

/**
 * Command in order to move drone to the right.
 */
void Nymeria::turnRight(){
	nymeriaRoutine(T_RIGHT);
}

/**
 * Command in order to stop the drone's movement,
 * i.e. stay at current position.
 */
void Nymeria::stop(){
	nymeriaRoutine(STOP);
}

/**
 * Command in order to make the drone take off.
 */
void Nymeria::takeOff(){
	nymeriaRoutine(TAKEOFF);
}

/**
 * Command in order to make the drone land,
 * i.e. underneath current position.
 */
void Nymeria::land(){
	nymeriaRoutine(LAND);
}

/**
 * Command in order to make drone stop and
 * immediately land.
 */
void Nymeria::emergencyStop(){
	nymeriaRoutine(E_STOP);
}

/**
 * Command in order to increase the maximum
 * speed by 10%.
 */
void Nymeria::increaseMaxSpeed(){
	nymeriaRoutine(I_M_SPEED);
}

/**
 * Command in order to decrease the maximum
 * speed by 10%.
 */
void Nymeria::decreaseMaxSpeed(){
	nymeriaRoutine(D_M_SPEED);
}

/**
 * Command in order to increase the linear
 * speed by 10%.
 */
void Nymeria::increaseLinearSpeed(){
	nymeriaRoutine(I_L_SPEED);
}

/**
 * Command in order to decrease the linear
 * speed by 10%.
 */
void Nymeria::decreaseLinearSpeed(){
	nymeriaRoutine(D_L_SPEED);
}

/**
 * Command in order to increase the angular
 * speed by 10%.
 */
void Nymeria::increaseAngularSpeed(){
	nymeriaRoutine(I_A_SPEED);
}

/**
 * Command in order to decrease the angular
 * speed by 10%.
 */
void Nymeria::decreaseAngularSpeed(){
	nymeriaRoutine(D_A_SPEED);
}
