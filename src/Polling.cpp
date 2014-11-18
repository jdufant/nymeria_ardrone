#include <nymeria_ardrone/Polling.h>

int main(int argc, char **argv)
{
	std_msgs::Empty empty_msg;

	ros::init(argc, argv, "polling");

	ros::NodeHandle n("/nymeria_ardrone");

	ros::Publisher polling_pub = n.advertise<std_msgs::Empty>("nymeria_ardrone/state_obstacle", 10);
	int rate = 10;
	ros::Rate loop_rate(rate);
	while(ros::ok()){
		polling_pub.publish(empty_msg);

		// mutexStateDrone.lock();
			// ROS_WARN("polling done");
			if(n.hasParam("stateDrone"))
				n.setParam("stateDrone", 99);
		// mutexStateDrone.unlock();

		loop_rate.sleep();
	}

	return 0;
}

Polling::Polling(){};