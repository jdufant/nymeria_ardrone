#include <nymeria_ardrone/Polling.h>

Polling::Polling(){};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "polling");

	ros::NodeHandle n;

	int rate = 10;
	ros::Rate loop_rate(rate);

	while(ros::ok()){
		NymeriaMutexDrone::lock();
			if(n.hasParam("stateObstacle"))
				n.setParam("stateObstacle", NymeriaConstants::O_FRONT);
		NymeriaMutexDrone::unlock();

		loop_rate.sleep();
	}

	return 0;
}