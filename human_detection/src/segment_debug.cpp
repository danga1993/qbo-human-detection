// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"
#include "config.h"

#include "segmenter/segmenter.h"

int main(int argc, char** argv)
{

	cv::FileStorage file;
	std::vector<std::string> files; 

	// Segmentation candidates
	std::vector<candidate> candidates; 

	std::cout << "Loading image: " << argv[1] << std::endl;

	// load file
	file.open(argv[1], cv::FileStorage::READ);

	// load image to matrix
	cv::Mat img;
	file["puka"] >> img;	

	//displayImg(img);

	std::cout << "Segmenting image" << std::endl;

	// segment
	Segmenter::segment(img, candidates);

	
	// Display the segments
	/*
	for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		if( !it->erased )
			displayImg(it->im); 
	} */

	display_candidates(img.cols/ALPHA, img.rows/ALPHA, candidates); 	

	file.release(); 

	// Display all the images
	cv::waitKey(); 

	
	
}
