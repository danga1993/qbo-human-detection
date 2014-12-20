#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

#include "human_detection.h"


Human_Detector::Human_Detector() {

	ROS_INFO("Initialising Human Detector"); 

}


Human_Detector::~Human_Detector() {

	ROS_INFO("Human detector cleaning up"); 

}


// Runs through stored images and classifies each image
void Human_Detector::play() {

	ROS_INFO("Hello"); 

}


int main(int argc, char** argv)
{


	// Initialise ROS node
	ros::init(argc, argv, "Human_Detector");

	// Declare handle to node - Used to communicate with node
	ros::NodeHandle n; 

	Human_Detector hd;

	hd.play();

	ros::spin();

	return 0;

}

