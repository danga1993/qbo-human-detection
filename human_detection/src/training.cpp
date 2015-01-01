// training montage

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include "training.h"

training_collect::training_collect() : it_in(nh_), it_out(nh_)
{
	key_pressed = false;

	// subscribe to input feed and publish output feed
	sub_img = it_in.subscribe("/camera/depth/image", 1, &training_collect::imageCallback, this);

	// subscribe to key press feed
	sub_key = nh_.subscribe("keypress", 1, &training_collect::keypressCallback, this);

	// publish to RViz
	pub_out = it_out.advertise("/training_out", 1);

	// initialise frame counter
	frame_count = 0;
}

// called on keypress
void training_collect::keypressCallback(const std_msgs::String::ConstPtr& msg)
{
	key_pressed = true;
}

// called when a frame is published
void training_collect::imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

	static ros::Time current_time; 
	static ros::Time prev_run(0); 

	//if (key_pressed)
	//{ 

	current_time = ros::Time::now(); 

	if( current_time - prev_run > ros::Duration(5.0) ) {

		prev_run = current_time; 

		cv_bridge::CvImagePtr in_msg;

		// convert frame to OpenCV format
		try {
			in_msg = cv_bridge::toCvCopy(original_image);
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}

		double minval, maxval;
		cv::minMaxIdx(in_msg->image, &minval, &maxval);

		std::cout << "Minval: " << minval << " Maxval" << maxval << std::endl;

		// write frame to a file
		char file_name[100];
		sprintf(file_name, "pons/%d.mat", frame_count);
		cv::FileStorage file(file_name, cv::FileStorage::WRITE);
		file << "puka" << in_msg->image;

		// write to output stream
		pub_out.publish(original_image);

		// increment counter
		frame_count++;

		key_pressed = false;

	}
}

int main(int argc, char** argv)
{
	// initialise ROS
	ros::init(argc, argv, "training");
	
	training_collect pons;

	// infinite loop
	ros::spin();

	return 0;
}


