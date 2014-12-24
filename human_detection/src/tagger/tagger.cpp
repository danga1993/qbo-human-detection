#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/merge_and_filter/candidate.h"

#include "tagger/tagger.h"


// Allows user to manually tag each candidate
void Tagger::tag(std::vector<candidate>& candidates) {

	// Create window for displaying candidates
	cv::namedWindow("CANDIDATES", CV_WINDOW_NORMAL);

	// Loop through and show candidates to user
	for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
	
		if( !it->erased ) {

			cv::imshow("CANDIDATES", it->im);

			// User response
			int resp = cv::waitKey(); 

			// Tag as human if user presses y
			it->human = ( resp == 'y' ) ? true : false; 

		}

	}

}


// Automatically tags candidates based on stored bounding boxes
void Tagger::tag(std::vector<candidate>& candidates, std::vector<cv::Rect>& bounding_boxes)  {

	// 

}


