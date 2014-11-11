#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


void callback(const ros::TimerEvent& event){
	std_msgs::String msg;
	std::stringstream ss;
	ss << "hello world " << count;
	msg.data = ss.str();
	ROS_INFO("%s", msg.data.c_str());
	chatter_pub.publish(msg);
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "polling");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
	int rate = 10;
	ros::Timer ros::NodeHandle::createTimer(ros::Duration (1/rate), <callback>, false);
	ros::Rate loop_rate(rate);
	int count = 0
	return 0;
}