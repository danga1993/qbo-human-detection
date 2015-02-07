#include <vector>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "labeller/labeller.h"

#include "config.h"


Labeller::Labeller() {

	// Initialize openCV window
	cv::namedWindow("Labeller", CV_WINDOW_AUTOSIZE);

	// Zero the active label
	active_label = cv::Rect(0,0,0,0); 

	// Hide active rectangle until drawing
	display_active = false; 

}

// Dummy function for mouse callback
void mouseprocess_dummy(int, int, int, int, void*) {
}
			
// Loads new frame into labeller
void Labeller::set_frame(cv::Mat& in_img) {

	img = in_img.clone();

	// Scale image for viewing contrast
	double min, max;
	cv::minMaxIdx(img, &min, &max); 
	img.convertTo(img, CV_8UC1, 255.0/max); 
	cv::cvtColor(img, img, CV_GRAY2BGR); 
	
	// Clear label rectangles
	labels_positive.clear(); 
	labels_negative.clear(); 

	// Zero the active label
	active_label = cv::Rect(0,0,0,0); 

	// Hide active rectangle until drawing
	display_active = false; 
	
}

// Allows user to label humans in the frame
void Labeller::label_frame() {

	// Activate callback
	cv::setMouseCallback("Labeller", Labeller::mouseprocess, (void*)this); 

	while( true ) {

		// Image to draw on
		img_draw = img.clone(); 

		// Add rectangles
		for( std::vector<cv::Rect>::iterator rect = labels_positive.begin(); rect != labels_positive.end(); rect++ )
			cv::rectangle(img_draw, *rect, CV_RGB(0,255,0), 2);

		for( std::vector<cv::Rect>::iterator rect = labels_negative.begin(); rect != labels_negative.end(); rect++ )
			cv::rectangle(img_draw, *rect, CV_RGB(255,0,0), 2);

		// Draw active rectangle
		if( display_active )
			cv::rectangle(img_draw, active_label, CV_RGB(255,255,255), 2);	
	
		// Draw window
		imshow("Labeller", img_draw); 

		// Process callbacks
		char c = cv::waitKey(15); 

		// Process keypress
		if( keyprocess(c) == 0 ) 
			break; 

	}

	// Deactivate callback
	cv::setMouseCallback("Labeller", mouseprocess_dummy); 

}


// Process mouse event
void Labeller::mouseprocess(int event, int x, int y, int flags, void * instance) {

	Labeller * labeller = (Labeller*) instance; 
	cv::Rect& active_label = labeller->active_label; 

	// Make sure coordinates cannot exceed bounds
	x = (x >= 0) ? x : 0; 
	y = (y >= 0) ? y : 0; 

	switch( event ) {

		case CV_EVENT_LBUTTONDOWN: 

			// Start drawing rectangle
			active_label = cv::Rect(x, y, 0, 0); 
			labeller->display_active = true; 

			break; 

		case CV_EVENT_MOUSEMOVE: 

			if( flags == CV_EVENT_FLAG_LBUTTON )
				active_label = cv::Rect(active_label.tl(), cv::Point(x, y)); 
			
			break; 

	}

}


// Process keypress events
int Labeller::keyprocess(char c) {

	if( c == -1 ) 
		return 1;

	// Save active label (positive)
	if( c == 'p' ) {

		// Stop drawing rectangle
		labels_positive.push_back(active_label); 
		display_active = false; 

	}

	// Save active label (negative)
	if( c == 'n' ) {

		// Stop drawing rectangle
		labels_negative.push_back(active_label); 
		display_active = false; 

	}

	// Quit
	if( c == 'q' ) 
		return 0; 

	return 1;

}


void Labeller::get_labels(std::vector<cv::Rect>& label_out_positive, std::vector<cv::Rect>& label_out_negative) { 

	label_out_positive = labels_positive; 
	label_out_negative = labels_negative; 

}

	




