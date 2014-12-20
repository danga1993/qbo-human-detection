// key press node

#include <iostream>
#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
	// initialise ROS
	ros::init(argc, argv, "keypress");

	ros::NodeHandle nh_;
	ros::Publisher pub_ = nh_.advertise<std_msgs::String>("keypress", 1000);

	while(ros::ok())
	{
		// wait for a key press
		char p[100];
		std::cin.getline(p, 100);

		if (strcmp(p, "exit") == 0)
			break;

		// publish message to server
		std_msgs::String msg;
		msg.data = p;
		pub_.publish(msg);

		ros::spinOnce();
	}

	return 0;
}
