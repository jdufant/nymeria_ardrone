//#include <nymeria_ardrone/NymeriaTest.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string.h>

#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/UInt8.h"
#include <ardrone_autonomy/Navdata.h>
#include <nymeria_ardrone/Nymeria.h>


class NymeriaTest
{
	public:
		NymeriaTest();
		ros::NodeHandle * getNH();
		void loop(ros::NodeHandle * n);
	private:
		ros::NodeHandle nh;
};


ros::NodeHandle * NymeriaTest::getNH(){
	return &nh;
}

NymeriaTest::NymeriaTest(){}


/**
 * Central functionality of the NymeriaTest: trigger Nymeria in order to actiate
 * obstacle detection and avoidance.
 **/
void NymeriaTest::loop(ros::NodeHandle * n){

	// TODO augment it to argument
	int rate = 50;
	int errorCode;
	int lastErrorCode = 0;
	
	//Param tests
	double paramSecDist = 0;
	double paramStateObs = 0;
	double paramFactor = 0.0;
	int paramCommand = 0;
	
	//Data to collect after testing
	double resultFactor = 0.0;
	int resultCmd = 0;

	int i(0);
	bool loopExit(false);

	Nymeria nym (n);
	
	//Take off for testing
	// n->setParam("nymeriaCommand", 10);
	

	std::fstream myfile;
	myfile.open("/home/grm/catkin_ws/src/nymeria_ardrone/test/test.txt", std::fstream::in );
	std::string line;

	if (myfile.good()){
		printf("file opened\n");
		myfile.clear();
		myfile.seekg(0, std::ios::beg);
	}	
	else
		printf("file error\n");


	while(!myfile.eof() && !loopExit)
	{
		
		int valChar = 0;

		printf("\n--- Test %d ---\n",++i);

		//get the line and convert it into stringstream for parsing
		std::getline(myfile, line);
		std::stringstream strs(line);

		// Read the integers using the operator >> 
		// numbers are separated by the tabulation (\t) and attributed to each variable
		strs >> paramSecDist >> paramCommand >> paramStateObs >> paramFactor;

		printf("dist = %f\t command = %d\t state = %f\t factor = %f\t\n", paramSecDist, paramCommand, paramStateObs, paramFactor);
		
		//Begin of test : Set params
		n->setParam("nymeriaSecurityDist", paramSecDist);
		n->setParam("nymeriaCommand", paramCommand);
		n->setParam("nymeriaStateObstacle", paramStateObs);
		n->setParam("nymeriaFactor", paramFactor);

		ros::Duration(1).sleep();
		
		//End of test : Collect data
		n->getParam("nymeriaFactor" , resultFactor);
		printf("nymeriaFactor : %f\t",resultFactor);
		n->getParam("nymeriaCommand" , resultCmd);
		printf("nymeriaCommand : %d\n",resultCmd);		
		nym.stop();
		
		ros::Duration(1).sleep();
		while ((valChar = getchar()) != '\n' && valChar != EOF)
			if (valChar == 'q')
				loopExit = true;
	}
	
	//Land
	n->setParam("nymeriaCommand", 11);
	
	myfile.close();
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "NymeriaTest");
	
	NymeriaTest si;

	si.loop(si.getNH());

	return 0;
}
