// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"
#include "tagger/tagger.h"
#include "config.h"

#include "segmenter/segmenter.h"

int main(int argc, char** argv)
{

	// Segmentation candidates
	std::vector<candidate> candidates; 

	// Bouding boxes for pre-tagging candidates
	std::vector<cv::Rect> bounding_boxes; 

	if (argc != 2)
		std::cout << "No file specified" << std::endl;

	// load file
	cv::FileStorage file(argv[1], cv::FileStorage::READ);

	// load image to matrix
	cv::Mat img;
	file["puka"] >> img;	

	// segment
	Segmenter::segment(img, candidates);

	// Tag the candidates
	if( TAG_AUTO ) {

		// Retrieve bounding boxes for image
		if( file["boundingbox"].type() == cv::FileNode::USER ) {
			file["boundingbox"] >> bounding_boxes; 
		} else {
			std::cout << "No bounding box defined for candidate" << std::endl; 
		}

		Tagger::tag(candidates, bounding_boxes); 

	} else {

		Tagger::tag(candidates); 

	}
	

	// Save the candidates
	std::stringstream fname; 
	int i = 0; 

	for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {

		if( !it->erased ) {

			// Generate filename
			fname.str(""); 
			fname << "candidates/" << i << ".mat"; 

			// Open file
		 	file = cv::FileStorage(fname.str(), cv::FileStorage::WRITE);

			file << "image" << it->im; 
			file << "human" << it->human;

			i++; 
		
		}

	}
		

}
