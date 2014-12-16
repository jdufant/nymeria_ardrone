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

/* PUBLIC methods */

/* Constructors */
/**
 * Default empty constructor.
 */
Nymeria::Nymeria(){};
/**
 * Constructor in order to create a meaningful object of the type Nymeria. Meaningful in terms of functionality:
 * It provides all navigation commands for the drone whilst ensuring obstacle protection and avoidance.
 */
Nymeria::Nymeria(ros::NodeHandle * n,  int securityDist){

	speed = 0.05;
	lastCmd = 0;

	/* Initialize move_msg. */
	init_move_msg();

	/* Initialize safeActions. */
	init_safeActions();
	
	/* Adapt node handle. */
	nh = n;

	/* Set parameters shared with all ROS nodes. */
	init_rosParams(securityDist);
	
	/* Initialize publishers. */
	init_publishers();

	//sub_navdata = nh->subscribe("/ardrone/navdata", 10, stateDroneCallback);

};

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
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::I_M_SPEED);
	NymeriaMutexCommand::unlock();
}
/**
 * Command in order to decrease the maximum
 * speed by 10%.
 */
void Nymeria::decreaseMaxSpeed(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::D_M_SPEED);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to increase the linear
 * speed by 10%.
 */
void Nymeria::increaseLinearSpeed(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::I_L_SPEED);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to decrease the linear
 * speed by 10%.
 */

void Nymeria::decreaseLinearSpeed(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::D_L_SPEED);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to increase the angular
 * speed by 10%.
 */
void Nymeria::increaseAngularSpeed(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::I_A_SPEED);
	NymeriaMutexCommand::unlock();
}

/**
 * Command in order to decrease the angular
 * speed by 10%.
 */
void Nymeria::decreaseAngularSpeed(){
	NymeriaMutexCommand::lock();
	nh->setParam("nymeriaCommand", NymeriaConstants::D_A_SPEED);
	NymeriaMutexCommand::unlock();
}


/* PRIVATE methods */
/**
 * Helper function in order to forward user's command
 * to drone.
 * Sets drone's navigation state in transmitted message
 *
 * @param data - navigation state.
 */

// void Nymeria::stateDroneCallback (const ardrone_autonomy::Navdata& data){
//  	printf("%f\n", data.rotY);
// }

/**
 * Helper function in order to initialize the array of safe actions.
 */
void Nymeria::init_safeActions(){
	this->safeActions[0] = NymeriaConstants::M_BACKWARD;
	this->safeActions[1] = NymeriaConstants::M_LEFT;
	this->safeActions[2] = NymeriaConstants::M_RIGHT;
	this->safeActions[3] = NymeriaConstants::M_UP;
	this->safeActions[4] = NymeriaConstants::M_DOWN;
	this->safeActions[5] = NymeriaConstants::T_LEFT;
	this->safeActions[6] = NymeriaConstants::T_RIGHT;
	this->safeActions[7] = NymeriaConstants::STOP;
	this->safeActions[8] = NymeriaConstants::TAKEOFF;
	this->safeActions[9] = NymeriaConstants::LAND;
	this->safeActions[10] = NymeriaConstants::E_STOP;
	this->safeActions[11] = NymeriaConstants::I_M_SPEED;
	this->safeActions[12] = NymeriaConstants::D_M_SPEED;
	this->safeActions[13] = NymeriaConstants::I_L_SPEED;
	this->safeActions[14] = NymeriaConstants::D_L_SPEED;
	this->safeActions[15] = NymeriaConstants::I_A_SPEED;
	this->safeActions[16] = NymeriaConstants::D_A_SPEED;
}

/**
 * Helper function in order to initialize ROS parameters nymeriaCommand, nymeriaStateObstacle, nymeriaSecurityDist.
 * @throws NymeriaInvalidSecurityDistance if entered security distance is negative.
 */
void Nymeria::init_rosParams(int securityDist){
	int tmpSecurityDist = -1;
	char nymeriaSecurityDist[] = "/nymeriaSecurityDist";
	/* nymeriaCommand */
	NymeriaMutexCommand::lock();
		nh->setParam("nymeriaCommand", 0);
	NymeriaMutexCommand::unlock();

	/* nymeriaStateObstacle */
	NymeriaMutexObstacle::lock();
		nh->setParam("nymeriaStateObstacle", -1);
	NymeriaMutexObstacle::unlock();
	
	try {
		/* nymeriaSecurityDist */
		if(securityDist >= 0){
			if(nh->hasParam("/nymeriaSecurityDist") && (tmpSecurityDist = getParameter(nymeriaSecurityDist))){
			
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
			fprintf(stderr, "%s", error.what());
		}
}

/**
 * Helper function in order to initialize move_msg.
 */
void Nymeria::init_move_msg(){
	this->move_msg.linear.x = 0;
	this->move_msg.linear.y = 0;
	this->move_msg.linear.z = 0;
	this->move_msg.angular.x = 0;
	this->move_msg.angular.y = 0;
	this->move_msg.angular.z = 0;
}

/**
 * Helper function in order to initialize publishers.
 */
void Nymeria::init_publishers(){
	this->pub_cmd_takeoff = nh->advertise<std_msgs::Empty>("ardrone/takeoff", 10);
	this->pub_cmd_land = nh->advertise<std_msgs::Empty>("ardrone/land", 10);
	this->pub_cmd_reset = nh->advertise<std_msgs::Empty>("ardrone/reset", 10);
	this->pub_cmd_move = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);
}
 
/**
 * Helper function in order to access ROS parameters (read access).
 * @param str - name of parameter.
 * @return read parameter value, -1 if no parameter is found.
 */
double Nymeria::getParameter(char * str){
	double param = 1.0;
	try{
		if(nh->getParam(str , param)){}
		else
			throw NymeriaParamExc(str);

	} catch(NymeriaExceptions& error){
		/* Display error message. */
		// TODO: wrap as ROS msg
		fprintf(stderr, "%s", error.what());
	}
	return param;
}

/**
 * Entry point of obstactle detection and avoidance.
 * Algorithm analyzes sensor data in the form of distances and decides whether to 
 * (1) either stop the drone immediately and let it move backward if applicable
 * (2) or let the drone slow down
 * (3) or process the user's command without acting.
 * @return constant representing processed command or -1, when there has been an obstacle.
 */
int Nymeria::validateStates(){
	int tmpCommand;
	char nymeriaCommand[] = "/nymeriaCommand";
	tmpCommand = getParameter(nymeriaCommand);

	/* Moving forward and obstacle possibly in front? */
	if (!isSafeAction(tmpCommand) && obstaclePossible()){
		/* (1)Security distance already passed? */
		if(underSecurityDist()){
			reactionRoutine();
			return NymeriaConstants::O_FRONT;	
		}
		/* (2)Anticipating obstacle. */
		else {
			slowDown();
			return NymeriaConstants::SLOW_DOWN; // TODO maybe add a new constant SLOW_DOWN
		}
		
	}
	/* (3)Forward command. */
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
 * 	   false: no, check for obstacles is necessary.
 */
bool Nymeria::isSafeAction(int cmd){
	for (int i = 0; i < sizeof(safeActions)/sizeof(*safeActions); i++){
		if (safeActions[i] == cmd){
			return true;
		}
	}
	return false;
}

/**
 * Is it possible, that there is an obstacle in front?
 * @return true: yes, obstacle anticipated.
 * 	   false: no, no obstacle to be likely in front.
 */
bool Nymeria::obstaclePossible(){
	//TODO : harcoded if actual dist < security dist +50
	char nymeriaStateObstacle[] = "/nymeriaStateObstacle";
	char nymeriaSecurityDist[] = "/nymeriaSecurityDist";
	return  ((getParameter(nymeriaStateObstacle) < (getParameter(nymeriaSecurityDist) + 50))
	       &&(getParameter(nymeriaStateObstacle) >= 0.0));
}

/**
 * Is there an obstacle in front closer than the given security distance?
 * @return true: yes, obstacle in front too close.
 * 	   false: no, security distance still kept.
 */
bool Nymeria::underSecurityDist(){
	int tmpStateObstacle = -1;
	int tmpSecurityDist = -1;
	char nymeriaStateObstacle[] = "/nymeriaStateObstacle";
	char nymeriaSecurityDist[] = "/nymeriaSecurityDist";
	
	tmpSecurityDist = getParameter(nymeriaSecurityDist);
	tmpStateObstacle = getParameter(nymeriaStateObstacle);

	return ((tmpStateObstacle < tmpSecurityDist)
		&& (tmpStateObstacle >= 0.0));	
}

/**
 * Forward command to drone.
 * @param cmd - incoming command.
 * @param factor - regulating speed factor for slow down, 1 by default.
 * @return constant representing cmd processed.
 */
int Nymeria::triggerAction(int cmd, double factor){

	/* Ignore commands that are repeatedly entered by the user and processed directly,
	   see validateStates case 3. */
	if((cmd == lastCmd) && (factor == 1.0))
		return cmd;

	switch(cmd){
	case NymeriaConstants::INIT:
		break;
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
		init_move_msg();
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
	
	move_msg.linear.x *= speed*factor;
	move_msg.linear.y *= speed;
	move_msg.linear.z *= speed;
	move_msg.angular.x *= speed;
	move_msg.angular.y *= speed;
	move_msg.angular.z *= speed;

	printf("move_msg.linear.x = %f; factor = %f\n", move_msg.linear.x, factor);
	pub_cmd_move.publish(move_msg);
	ROS_INFO("Move :\nlinear.x = %f\nlinear.y = %f\nlinear.z = %f\nangular.x = %f\nangular.y = %f\nangular.z = %f\n", 
			move_msg.linear.x, move_msg.linear.y, move_msg.linear.z,
			move_msg.angular.x,move_msg.angular.y,move_msg.angular.z);

	/* Reinitialize move_msg. */
	init_move_msg();

	lastCmd = cmd;

	return cmd;
}

/**
 * Routine in order to make drone stop in front of obstacle and
 * keep the security distance.
 */
void Nymeria::reactionRoutine(){
	ROS_INFO("reaction routine");
	triggerAction(NymeriaConstants::STOP);
	keepSecurityDistance();
}

/**
 * Method in order to keep security distance by moving backward if necessary.
 */
void Nymeria::keepSecurityDistance(){
	int tmpCommand;
	char nymeriaCommand[] = "/nymeriaCommand";

	tmpCommand = getParameter(nymeriaCommand);

	if(underSecurityDist())
		triggerAction(NymeriaConstants::M_BACKWARD);
	
	while(underSecurityDist());
	
	triggerAction(NymeriaConstants::STOP);
}

/**
 * Method in order to initiate slowing down.
 */
void Nymeria::slowDown(){
	char nymeriaCommand[] = "/nymeriaCommand";
	char nymeriaFactor[] = "/nymeriaFactor";
	this->lastCmd = getParameter(nymeriaCommand);
	/* Not a safe action and over security distance? */
	while(!isSafeAction(this->lastCmd) && !underSecurityDist()){
		triggerAction(this->lastCmd, getParameter(nymeriaFactor));
		this->lastCmd = getParameter(nymeriaCommand);
	}
	return;
}
