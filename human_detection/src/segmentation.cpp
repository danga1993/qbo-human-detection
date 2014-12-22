// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"

#include "segmenter/segmenter.h"

int main(int argc, char** argv)
{

	// Segmentation candidates
	std::vector<candidate> candidates; 

	if (argc != 2)
		std::cout << "No file specified" << std::endl;

	// load file
	cv::FileStorage file(argv[1], cv::FileStorage::READ);

	// load image to matrix
	cv::Mat img;
	file["puka"] >> img;	

	// segment
	Segmenter::segment(img, candidates);

	// Display the candidates
	/*for(std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
		 displayImg(it->im); 
	}*/

	displayImg(img); 

	cv::waitKey();

}
