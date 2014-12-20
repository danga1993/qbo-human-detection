#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <vector>
#include <iostream>
#include <fstream>

const char *imdir = "../spinello/"; 

int main(int argc, char** argv)
{


	// Initialise ROS node
	ros::init(argc, argv, "Segmenter");

	// Declare handle to node - Used to communicate with node
	ros::NodeHandle n; 

	Segmenter seg;

	// Load images
	

	// Loop through images and output segmented versions
	


	ros::spin();

	return 0;

}

