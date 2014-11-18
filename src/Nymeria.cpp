#include <nymeria_ardrone/Nymeria.h>
#include <nymeria_ardrone/NymeriaParamExc.h>

/**
 * Definition of the class Nymeria, that defines all functionalities
 * in order to allow for drone navigation with obstacle detection and avoidance.
 */

/* PRIVATE methods */
/**
 * Helper function in order to forward user's command
 * to drone.
 * Sets drone's navigation state in transmitted message
 *
 * @param data - navigation state.
 */

// void Nymeria::stateDroneCallback (const ardrone_autonomy::Navdata& data){
// 	navData.state = data.state;
// }

Nymeria::Nymeria(){};
Nymeria::Nymeria(ros::NodeHandle * n)   
{
	speed = 0.2;

	move_msg.linear.x = 0;
	move_msg.linear.y = 0;
	move_msg.linear.z = 0;
	move_msg.angular.x = 0;
	move_msg.angular.y = 0;
	move_msg.angular.z = 0;

	/* Initialize safeActions. */
	safeActions[0] = NymeriaConstants::M_BACKWARD;
	safeActions[1] = NymeriaConstants::M_LEFT;
	safeActions[2] = NymeriaConstants::M_RIGHT;
	safeActions[3] = NymeriaConstants::M_UP;
	safeActions[4] = NymeriaConstants::M_DOWN;
	safeActions[5] = NymeriaConstants::T_LEFT;
	safeActions[6] = NymeriaConstants::T_RIGHT;
	safeActions[7] = NymeriaConstants::STOP;
	safeActions[8] = NymeriaConstants::TAKEOFF;
	safeActions[9] = NymeriaConstants::LAND;
	safeActions[10] = NymeriaConstants::E_STOP;
	safeActions[11] = NymeriaConstants::I_M_SPEED;
	safeActions[12] = NymeriaConstants::D_M_SPEED;
	safeActions[13] = NymeriaConstants::I_L_SPEED;
	safeActions[14] = NymeriaConstants::D_L_SPEED;
	safeActions[15] = NymeriaConstants::I_A_SPEED;
	safeActions[16] = NymeriaConstants::D_A_SPEED;

	nh = n;

	/* Set parameters shared with all ROS nodes. */ 
	// nh->setParam("nymeria/mutexStateDrone", true);
	// nh->setParam("nymeria/mutexStateObstacle", true);
	nh->setParam("stateDrone", 0);
	nh->setParam("stateObstacle", 0);

	pub_cmd_takeoff = nh->advertise<std_msgs::Empty>("ardrone/takeoff", 10);
	pub_cmd_land = nh->advertise<std_msgs::Empty>("ardrone/land", 10);
	pub_cmd_reset = nh->advertise<std_msgs::Empty>("ardrone/reset", 10);
	pub_cmd_move = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);

	// sub_navdata = nh->subscribe("ardrone/navdata", 10, stateDroneCallback);

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
	case NymeriaConstants::M_FORWARD:
		ROS_INFO("M_FORWARD");
		move_msg.linear.x = 1;
		break;
	case NymeriaConstants::M_BACKWARD:
		ROS_INFO("M_BACKWARD");
		move_msg.linear.x = -1;
		break;
	case NymeriaConstants::M_LEFT:
		ROS_INFO("M_LEFT");
		move_msg.linear.y = 1;
		break;
	case NymeriaConstants::M_RIGHT:
		ROS_INFO("M_RIGHT");
		move_msg.linear.y = -1;
		break;
	case NymeriaConstants::M_UP:
		ROS_INFO("M_UP");
		move_msg.linear.z = 1;
		break;
	case NymeriaConstants::M_DOWN:
		ROS_INFO("M_DOWN");
		move_msg.linear.z = -1;
		break;
	case NymeriaConstants::T_LEFT:
		ROS_INFO("T_LEFT");
		move_msg.angular.z = 1;
		break;
	case NymeriaConstants::T_RIGHT:
		ROS_INFO("T_RIGHT");
		move_msg.angular.z = -1;
		break;

	case NymeriaConstants::STOP:
		ROS_INFO("STOP");
		move_msg.linear.x = 0;
		move_msg.linear.y = 0;
		move_msg.linear.z = 0;
		move_msg.angular.x = 0;
		move_msg.angular.y = 0;
		move_msg.angular.z = 0;
		break;

	case NymeriaConstants::TAKEOFF:
		ROS_INFO("TAKEOFF");
		pub_cmd_takeoff.publish(empty_msg);
		while(navData.state == 6);
		break;
	case NymeriaConstants::LAND:
		ROS_INFO("LAND");
		pub_cmd_land.publish(empty_msg);
		while(navData.state == 8);
		break;
	case NymeriaConstants::E_STOP:
		ROS_INFO("E_STOP");
		pub_cmd_reset.publish(empty_msg);
		break;

	case NymeriaConstants::I_M_SPEED:
		speed += 0.1;
		break;
	case NymeriaConstants::D_M_SPEED:
		speed -= 0.1;
		break;
	case NymeriaConstants::I_L_SPEED:
		speed += 0.1;
		break;
	case NymeriaConstants::D_L_SPEED:
		speed -= 0.1;
		break;
	case NymeriaConstants::I_A_SPEED:
		speed += 0.1;
		break;
	case NymeriaConstants::D_A_SPEED:
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
	// between 1 and 12
	if(cmd >= 1 || cmd <= 12){

		NymeriaMutexDrone::lock();
			ROS_WARN("job done");
			if(nh->hasParam("stateDrone"))
				nh->setParam("stateDrone", cmd);
		NymeriaMutexDrone::unlock();
		
		return;
	}

	validateStates(NymeriaConstants::CHECK);
}

/**
 * Check whether current state of drone and current state of
 * obstacle are compatible with user's command.
 *
 * Recursive function, that terminates, when either there is
 * a conflict between states and input command (trigger
 * reactionRoutine) (Case 1) or
 * states and command are compatible and the command has been
 * sent the first time (forward command) (Case 2).
 * Otherwise: When none of the above cases apply, it means that
 * the drone is moving (forward) and program has to keep checking
 * states in order to detect obstacles (Case 3).
 *
 * @param cmd - incoming command: Either M_FORWARD (termination)
 * or CHECK (recursive function call to validateStates(CHECK).
 */
void Nymeria::validateStates(int cmd){
	int tmpStateDrone;
	int tmpStateObstacle;

	try{
		/* Get current state of drone. */
		if(nh->getParam("nymeria_ardrone/stateDrone", stateDrone))
			tmpStateDrone = stateDrone;
		else
			throw NymeriaParamExc();
		/* Get current state of obstacle. */
		if(nh->getParam("nymeria_ardrone/stateObstacle", stateObstacle))
			tmpStateObstacle = stateObstacle;
		else
			throw NymeriaParamExc();

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr,error.what());
	}


	/* Are the two states in conflict? (Case 1) */
	if(	   (tmpStateDrone == NymeriaConstants::M_FORWARD)
			&& (tmpStateObstacle == NymeriaConstants::O_FRONT)){
		reactionRoutine();
		// exit recursion/loop
		return;
	}
	else
		/* Command is being processed the first time. (Case 2) */
		if (cmd > 0){
			triggerAction(cmd);
			// exit recursion/loop
			return;
		}

	/*
	 * (Case 3)
	 * Recursive call --> Drone is moving forward.
	 * Keep checking states.
	 */
	validateStates(NymeriaConstants::CHECK);
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
	triggerAction(NymeriaConstants::STOP);
}

/* PUBLIC methods */

/**
 * Command in order to move drone forward.
 */
void Nymeria::moveForward(){
	nymeriaRoutine(NymeriaConstants::M_FORWARD);
}
/**
 * Command in order to move drone backward.
 */
void Nymeria::moveBackward(){
	nymeriaRoutine(NymeriaConstants::M_BACKWARD);
}
/**
 * Command in order to make drone rotate to the left.
 */
void Nymeria::moveLeft(){
	nymeriaRoutine(NymeriaConstants::M_LEFT);
}

/**
 * Command in order to make drone rotate to the right.
 */

void Nymeria::moveRight(){
	nymeriaRoutine(NymeriaConstants::M_RIGHT);
}
/**
 * Command in order to move drone upward,
 * i.e. increase altitude.
 */
void Nymeria::moveUp(){
	nymeriaRoutine(NymeriaConstants::M_UP);
}

/**
 * Command in order to move drone downward,
 * i.e. decrease altitude.
 */
void Nymeria::moveDown(){
	nymeriaRoutine(NymeriaConstants::M_DOWN);
}
/**
 * Command in order to move drone to the left.
 */
void Nymeria::turnLeft(){
	nymeriaRoutine(NymeriaConstants::T_LEFT);
}

/**
 * Command in order to move drone to the right.
 */
void Nymeria::turnRight(){
	nymeriaRoutine(NymeriaConstants::T_RIGHT);
}

/**
 * Command in order to stop the drone's movement,
 * i.e. stay at current position.
 */
void Nymeria::stop(){
	nymeriaRoutine(NymeriaConstants::STOP);
}
/**
 * Command in order to make the drone take off.
 */
void Nymeria::takeOff(){
	nymeriaRoutine(NymeriaConstants::TAKEOFF);
}
/**
 * Command in order to make the drone land,
 * i.e. underneath current position.
 */
void Nymeria::land(){
	nymeriaRoutine(NymeriaConstants::LAND);
}

/**
 * Command in order to make drone stop and
 * immediately land.
 */
void Nymeria::emergencyStop(){
	nymeriaRoutine(NymeriaConstants::E_STOP);
}
/**
 * Command in order to increase the maximum
 * speed by 10%.
 */
void Nymeria::increaseMaxSpeed(){
	nymeriaRoutine(NymeriaConstants::I_M_SPEED);
}
/**
 * Command in order to decrease the maximum
 * speed by 10%.
 */
void Nymeria::decreaseMaxSpeed(){
	nymeriaRoutine(NymeriaConstants::D_M_SPEED);
}

/**
 * Command in order to increase the linear
 * speed by 10%.
 */
void Nymeria::increaseLinearSpeed(){
	nymeriaRoutine(NymeriaConstants::I_L_SPEED);
}

/**
 * Command in order to decrease the linear
 * speed by 10%.
 */

void Nymeria::decreaseLinearSpeed(){
	nymeriaRoutine(NymeriaConstants::D_L_SPEED);
}

/**
 * Command in order to increase the angular
 * speed by 10%.
 */
void Nymeria::increaseAngularSpeed(){
	nymeriaRoutine(NymeriaConstants::I_A_SPEED);
}

/**
 * Command in order to decrease the angular
 * speed by 10%.
 */
void Nymeria::decreaseAngularSpeed(){
	nymeriaRoutine(NymeriaConstants::D_A_SPEED);
}

