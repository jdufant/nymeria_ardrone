#include <nymeria_ardrone/Nymeria.h>
#include <nymeria_ardrone/NymeriaParamExc.h>
#include <nymeria_ardrone/NymeriaInvalidSecurityDistance.h>
#include <nymeria_ardrone/NymeriaMutexCommand.h>
#include <nymeria_ardrone/NymeriaMutexObstacle.h>
#include <nymeria_ardrone/NymeriaMutexSecurityDistance.h>
#include <string.h>

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
Nymeria::Nymeria(ros::NodeHandle * n,  int securityDist){

	int tmpSecurityDist = -1;

	speed = 0.05;
	lastCmd = 0;

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
	
	NymeriaMutexCommand::lock();
		nh->setParam("nymeriaCommand", 0);
	NymeriaMutexCommand::unlock();

	NymeriaMutexObstacle::lock();
		nh->setParam("nymeriaStateObstacle", -1);
	NymeriaMutexObstacle::unlock();

	try {
		if(securityDist >= 0){
			if(nh->hasParam("/nymeriaSecurityDist") && (tmpSecurityDist = getParameter("/nymeriaSecurityDist"))){
				
				if(tmpSecurityDist != securityDist){
					ROS_WARN("Given security distance does not match security distance given in NymeriaCheckObstacle.");
					ROS_WARN("First security distance given will be considered.");
				}
			}
			else {
				NymeriaMutexSecurityDistance::lock();
				nh->setParam("nymeriaSecurityDist", securityDist);
				NymeriaMutexSecurityDistance::unlock();
			}
		}
		else
			throw NymeriaInvalidSecurityDistance();
	}
	catch(NymeriaExceptions& error){
		/* Display error message. */
		fprintf(stderr,error.what());
	}

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
 * Forward command to drone.
 * @param cmd - incoming command.
 */
int Nymeria::triggerAction(int cmd){

  float factor = 0.0;
	if(cmd == lastCmd)
		return cmd;

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
		// jjjwhile(navData.state == 6);
		break;
	case NymeriaConstants::LAND:
		ROS_INFO("LAND");
		pub_cmd_land.publish(empty_msg);
		// while(navData.state == 8);
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
	
	factor = calculateSpeedFactor();
	move_msg.linear.x *= speed*factor;
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

	lastCmd = cmd;

	return cmd;
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
int Nymeria::validateStates(){
	int tmpCommand;
	int tmpSecurityDist;
	int tmpStateObstacle;
	
	tmpCommand = getParameter("/nymeriaCommand");
	tmpSecurityDist = getParameter("/nymeriaSecurityDist");
	tmpStateObstacle = getParameter("/nymeriaStateObstacle");

	/* move forward and obstacle in front */
	if (!isSafeAction(tmpCommand) && hasObstacle()){
		reactionRoutine();
		return NymeriaConstants::O_FRONT;
	}
	/* forward command */
	else {
		return (triggerAction(tmpCommand));
	}
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
			// ROS_INFO("cas 6");

			return true;
		}
	}
	return false;
}

void Nymeria::keepSecurityDistance(){
	int tmpCommand;

	tmpCommand = getParameter("/nymeriaCommand");

	if(hasObstacle())
		triggerAction(NymeriaConstants::M_BACKWARD);
	
	while(hasObstacle());
	
	triggerAction(NymeriaConstants::STOP);
}

bool Nymeria::hasObstacle(){
	int tmpStateObstacle;
	int tmpSecurityDist;

	tmpSecurityDist = getParameter("/nymeriaSecurityDist");
	tmpStateObstacle = getParameter("/nymeriaStateObstacle");

	return ((tmpStateObstacle < tmpSecurityDist)
		&& (tmpStateObstacle >= 0));	
}



/*
 * Calcule un facteur multiplicatif qui adaptera la vitesse du drone à l'approche de l'obstacle
 *
 * @return factor
 */

float Nymeria::calculateSpeedFactor(){
  int tmpStateObstacle;
  int tmpSecurityDist;

  tmpSecurityDist = getParameter("/nymeriaSecurityDist");
  tmpStateObstacle = getParameter("/nymeriaStateObstacle");
  float factor = (tmpStateObstacle - tmpSecurityDist)/100;
  
  if(factor > 1.0){
    factor = 1.0;
  } else if (factor < 0.0){
    factor = 0.0;
  }

  return factor;  
}


/** TODO
 * Routine in order to make drone stop in front of obstacle and
 * then bypass it, i.e. navigate around it.
 */
void Nymeria::reactionRoutine(){
	ROS_INFO("reaction routine");
	triggerAction(NymeriaConstants::STOP);
	keepSecurityDistance();
}

/* PUBLIC methods */
int Nymeria::getParameter(char * str){
	int param;
	try{
		if(nh->getParam(str , param)){}
		else
			throw NymeriaParamExc(str);

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr, error.what());
	}
	return param;
}


/**
 * Command in order to move drone forward.
 */
void Nymeria::moveForward(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_FORWARD);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to move drone backward.
 */
void Nymeria::moveBackward(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_BACKWARD);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to make drone rotate to the left.
 */
void Nymeria::moveLeft(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_LEFT);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to make drone rotate to the right.
 */

void Nymeria::moveRight(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_RIGHT);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to move drone upward,
 * i.e. increase altitude.
 */
void Nymeria::moveUp(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_UP);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to move drone downward,
 * i.e. decrease altitude.
 */
void Nymeria::moveDown(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::M_DOWN);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to move drone to the left.
 */
void Nymeria::turnLeft(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::T_LEFT);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to move drone to the right.
 */
void Nymeria::turnRight(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::T_RIGHT);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to stop the drone's movement,
 * i.e. stay at current position.
 */
void Nymeria::stop(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::STOP);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to make the drone take off.
 */
void Nymeria::takeOff(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::TAKEOFF);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to make the drone land,
 * i.e. underneath current position.
 */
void Nymeria::land(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::LAND);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to make drone stop and
 * immediately land.
 */
void Nymeria::emergencyStop(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::E_STOP);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to increase the maximum
 * speed by 10%.
 */
void Nymeria::increaseMaxSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::I_M_SPEED);
}
/**
 * Command in order to decrease the maximum
 * speed by 10%.
 */
void Nymeria::decreaseMaxSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::D_M_SPEED);
}

/**
 * Command in order to increase the linear
 * speed by 10%.
 */
void Nymeria::increaseLinearSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::I_L_SPEED);
}

/**
 * Command in order to decrease the linear
 * speed by 10%.
 */

void Nymeria::decreaseLinearSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::D_L_SPEED);
}

/**
 * Command in order to increase the angular
 * speed by 10%.
 */
void Nymeria::increaseAngularSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::I_A_SPEED);
}

/**
 * Command in order to decrease the angular
 * speed by 10%.
 */
void Nymeria::decreaseAngularSpeed(){
	nh->setParam("nymeriaCommand", NymeriaConstants::D_A_SPEED);
}

