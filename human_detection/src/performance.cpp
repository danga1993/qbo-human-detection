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

	// File for loading
	cv::FileStorage file;
	std::vector<std::string> files; 
	cv::Mat img; 

	// Bounding boxes
  std::vector<cv::Rect> bounding_boxes; 
	std::vector<candidate> candidates; 

	// Retrieve test data frames 
	if( TEST_TAG_AUTO ) 

		// Get list of files
		directory_list(files, "test_data"); 

		// Loop through 
		for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

			file.open(*it, cv::FileStorage::READ); 
			if( !file.isOpened ) {
				std::cout << "Failed to open file " << *it << std::endl; 

			// Load the frame data
			file["puka"] >> img; 
			file["boundingboxes"] >> bounding_boxes; 

			// Segment the frame
			Segmenter::segment(img, candidates);

			// Tag the candidates
			Tagger::tag(candidates, bounding_boxes); 

		 	for (std::vector<candidate>::iterator it_cand = candidates.begin(); it_cand != files.end(); it_cand++) {

				// Extract features
				
				// Run each candidate through classifier

				// Classify and store correct/not correct

			}

		} else if ( !TEST_TAG_AUTO ) {

				// loop through candidates

				// Extract features

				// Run candidate through classifier

				// Classify and store correct/not correct

		}
		

}



