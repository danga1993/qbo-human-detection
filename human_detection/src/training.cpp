// training montage

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <image_transport/image_transport.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include "helper/image.h"

#include "training.h"

training_collect::training_collect() : it_(nh_), sub_img_depth(it_, "/camera/depth_registered/image_raw", 1), sub_img_colour(it_, "/camera/rgb/image_color", 1), sync(DepthSyncPolicy(10), sub_img_depth, sub_img_colour)
{
	key_pressed = false;

	// subscribe to input feed and publish output feed
	// sub_img = it_in.subscribe("/camera/depth/image", 1, &training_collect::imageCallback, this);
	sync.registerCallback( boost::bind( &training_collect::imageCallback, this, _1, _2 ) ); 	

	// subscribe to key press feed
	sub_key = nh_.subscribe("keypress", 1, &training_collect::keypressCallback, this);

	// publish to RViz
	pub_out = it_.advertise("/training_out", 1);

	// initialise frame counter
	frame_count = 0;

	std::cout << "Waiting for frames" << std::endl;

}

// called on keypress
void training_collect::keypressCallback(const std_msgs::String::ConstPtr& msg)
{
	key_pressed = true;
}

// called when a frame is published
void training_collect::imageCallback(const sensor_msgs::ImageConstPtr& depth_image, const sensor_msgs::ImageConstPtr& colour_image)
{

	static ros::Time current_time; 
	static ros::Time prev_run(0); 

	//if (key_pressed)
	//{ 

	current_time = ros::Time::now(); 

	if( current_time - prev_run > ros::Duration(0.5) ) {

		prev_run = current_time; 

		cv_bridge::CvImagePtr in_depth;
		cv_bridge::CvImagePtr in_colour;

		// convert frame to OpenCV format
		try {
			in_depth = cv_bridge::toCvCopy(depth_image);
			in_colour = cv_bridge::toCvCopy(colour_image); 
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("ROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
			return;
		}

		std::cout << "Collected frames" << std::endl;

		double minval, maxval;
		cv::minMaxIdx(in_depth->image, &minval, &maxval);

		// std::cout << "Minval: " << minval << " Maxval" << maxval << std::endl;

		std::ostringstream dir_name; 
		dir_name << "frames/" << frame_count; 

		// Make directory if it does not exist
		make_directory(dir_name.str()); 

		/*
		// write frame to a file
		char file_name[100];
		sprintf(file_name, "frames/%d/img.mat", frame_count);
		cv::FileStorage file(file_name, cv::FileStorage::WRITE);
		file << "puka" << in_msg->image; */

		// Depth is Uint16 in mm - Convert to float (m)
		cv::Mat depth_im; 
		(in_depth->image).convertTo(depth_im, CV_32FC1, 1.0/1000); 

		// Write output pngs
		image_write(dir_name.str() + "/depth.png", depth_im); 
		image_write(dir_name.str() + "/colour.png", in_colour->image); 

		std::cout << "Saved frames" << std::endl;

		// Publish output stream
		pub_out.publish(depth_image);

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


