#include <vector>
#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"
#include "helper/image.h"

#include "seg_lib/merge_and_filter/candidate.h"

#include "tagger/tagger.h"


// Allows user to manually tag each candidate
void Tagger::tag(std::vector<candidate>& candidates) {

	// Create window for displaying candidates
	cv::namedWindow("CANDIDATES", CV_WINDOW_NORMAL);

	// Loop through and show candidates to user
	for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
	
		if( !it->erased ) {

			displayImg(it->im, "CANDIDATES");

			// User response
			int resp = cv::waitKey(); 

			// Tag as human if user presses y
			it->human = ( resp == 'y' ) ? true : false; 

		}

	}

}


// Automatically tags candidates based on stored bounding boxes
void Tagger::tag(std::vector<candidate>& candidates, std::vector<cv::Rect>& bounding_boxes)  {

	for( std::vector<candidate>::iterator candidate = candidates.begin(); candidate != candidates.end(); candidate++ ) {

		if( candidate->erased ) 
			continue; 

		// Check for sufficient overlap between candidate and rectangle
		for( std::vector<cv::Rect>::iterator box = bounding_boxes.begin(); box != bounding_boxes.end(); box++ ) {

			candidate->human = candidate_intersect(*candidate, *box); 

		}
	
		if( candidate->human ) 
			std::cout << "Candidate is human: " << candidate->human << std::endl;

		// Show candidate
		//displayImg(candidate->im, "CANDIDATES");

		// User response
		//int resp = cv::waitKey(); 

	}

}


bool Tagger::candidate_intersect(candidate& cand, cv::Rect& box) {

	// Area of intersection
	int area_intersect = (cand.box & box).area(); 
	int area_total = cand.box.area() + box.area() - area_intersect;

	// std::cout << "Candidate: " << cand.box << " Box: " << box << std::endl;
  //std::cout << "Intersect area: " << area_intersect << " Total area: " << area_total << std::endl;

	// Check for required intersection fraction
	if( (float)area_intersect / area_total > MIN_INTERSECT_RATIO )
		return true; 
	else
		return false; 

}


