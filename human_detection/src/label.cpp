#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"
#include "helper/image.h"

#include "labeller/labeller.h"

#include "config.h"

int main(int argc, char** argv)
{
	
	cv::FileStorage file;
	std::vector<std::string> files; 

	cv::Mat img; 
	std::vector<cv::Rect> labels_positive; 
	std::vector<cv::Rect> labels_negative; 

	Labeller label; 
	
	// Get list of files
	directory_list(files, "frames"); 

	std::cout << "Files: " << files.size() << std::endl;

	// Loop through 
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

		image_read(*it + "/depth.png", img); 
		
		// Label the image
		label.set_frame(img); 
		label.label_frame();
		label.get_labels(labels_positive, labels_negative); 

		// Remake the file
		remove((*it + "/data.mat").c_str()); 
	
		file.open((*it + "/data.mat"), cv::FileStorage::WRITE); 

		// Save labels
		file << "bounding_positive" << labels_positive; 
		file << "bounding_negative" << labels_negative; 

		file.release();

	}

	return 0;
}


