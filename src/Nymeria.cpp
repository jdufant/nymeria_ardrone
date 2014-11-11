#include "Nymeria.h"

void stateDroneCallback (const ardrone_autonomy::Navdata& data){
	navData.state = data.state;
}

void Nymeria::nymeriaRoutine(int cmd){
	if(isSafeAction(cmd))
		triggerAction(cmd);
	else
		validateStates(cmd);
}

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
*
*
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

void Nymeria::reactionRoutine(){}

void Nymeria::moveForward(){
	nymeriaRoutine(M_FORWARD);
}
void Nymeria::moveBackward(){
	nymeriaRoutine(M_BACKWARD);
}
void Nymeria::moveLeft(){
	nymeriaRoutine(M_LEFT);
}
void Nymeria::moveRight(){
	nymeriaRoutine(M_RIGHT);
}
void Nymeria::moveUp(){
	nymeriaRoutine(M_UP);
}
void Nymeria::moveDown(){
	nymeriaRoutine(M_DOWN);
}
void Nymeria::turnLeft(){
	nymeriaRoutine(T_LEFT);
}
void Nymeria::turnRight(){
	nymeriaRoutine(T_RIGHT);
}
void Nymeria::stop(){
	nymeriaRoutine(STOP);
}
void Nymeria::takeOff(){
	nymeriaRoutine(TAKEOFF);
}
void Nymeria::land(){
	nymeriaRoutine(LAND);
}
void Nymeria::emergencyStop(){
	nymeriaRoutine(E_STOP);
}
void Nymeria::increaseMaxSpeed(){
	nymeriaRoutine(I_M_SPEED);
}
void Nymeria::decreaseMaxSpeed(){
	nymeriaRoutine(D_M_SPEED);
}
void Nymeria::increaseLinearSpeed(){
	nymeriaRoutine(I_L_SPEED);
}
void Nymeria::decreaseLinearSpeed(){
	nymeriaRoutine(D_L_SPEED);
}
void Nymeria::increaseAngularSpeed(){
	nymeriaRoutine(I_A_SPEED);
}
void Nymeria::decreaseAngularSpeed(){
	nymeriaRoutine(D_A_SPEED);
}
