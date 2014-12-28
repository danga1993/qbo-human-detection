#include <vector>
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "feature/featurevector.h"
#include "feature/rdsfvector.h"

#include "config.h"


// Initialize vector by computing required rectangle coordinates
RDSFVector::RDSFVector() : integral_images(DEPTH_BINS) {

  // Candidate dimensions in cells
  int cand_width_cells = CANDIDATE_WIDTH / RECT_CELL_SIZE; 
  int cand_height_cells = CANDIDATE_HEIGHT / RECT_CELL_SIZE;

  // Calculate square for each size (1x1 - 8x8)
  for( int side = 1; side <= RECT_MAX_DIM; side++ ) {

    for( int x = 0; x < cand_width_cells - (side-1); x++ ) {
      for( int y = 0; y < cand_height_cells - (side-1); y++ ) {

				RDSFRect rect(cv::Rect(x, y, side, side), &integral_images); 

        rectangles.push_back(rect);

      }
    }

  }

	int a1 = rectangles.size() - 1; 
	int sum = 0; 

	// Loop through and calculate boundaries of first rectangle in vector
	for( int n = 0; n < rectangles.size(); n++ ) {

		// Calculate sum of arithmetic progression
		sum += a1; 
		a1--; 
		
		// Place in vector
		rect_partitions.push_back(sum); 

	}

	// Calculate vector length
	length = (rectangles.size() * (rectangles.size()-1)) / 2;

}

int RDSFVector::getLength()
{
	return length;
}

     
// Computes integral images for each histogram bin
void RDSFVector::set_candidate(const candidate& cand) {

	std::vector<cv::Mat> hist_images(DEPTH_BINS+1); 
	cv::Size imsize(CANDIDATE_WIDTH, CANDIDATE_HEIGHT);
	cv::Mat cand_norm;

	// Normalise candidate image
	cv::normalize(cand.im, cand_norm, 0, 1, cv::NORM_MINMAX);

	// Fill first bin with 1's (because threshold is at 0)
	hist_images.at(0) = cv::Mat::ones(CANDIDATE_HEIGHT, CANDIDATE_WIDTH, CV_32FC1);

	// Separate image into histogram bins and calculate integral images for each bin
	for( int bin = 0; bin < DEPTH_BINS; bin++ ) {

		hist_images.at(bin+1) = cv::Mat(imsize, CV_32FC1);
		if( bin < DEPTH_BINS-1 ) {
			// Threshold the depth image
			cv::threshold(cand.im, hist_images.at(bin+1), (bin+1)/DEPTH_BINS, 1.0, cv::THRESH_BINARY);
		} else {
			// Last thresholded image all zeros
			hist_images.at(bin+1) = cv::Mat::zeros(imsize, CV_32FC1); 
		}

		// We might not need this if floats subtract to zero as required
		// hist_images[bin+1].convertTo(hist_images[bin+1], CV_8UC1);

		// Calculate histogram bin image (difference of thresholded images)
		hist_images.at(bin) = hist_images.at(bin) - hist_images.at(bin+1);

		// Calculate integral image
		cv::integral(hist_images.at(bin), integral_images.at(bin)); 	

	}

} 


// Returns details of feature with given id
void RDSFVector::getfeaturetype(int feature_id) {

}


// Retrieves vector of required features
void RDSFVector::getfeatures(std::vector<int> feature_ids, cv::Mat& features) {

  // Clear features matrix
  features = cv::Mat::zeros(rectangles.size() * rectangles.size() / 2, 1, CV_32FC1); 

  // If feature vector empty get all features
  if( feature_ids.empty() ) {

    // Loop through all combinations
    for( int rect1 = 0, i = 0; rect1 < rectangles.size(); rect1++ ) {
      for( int rect2 = rect1+1; rect2 < rectangles.size(); rect2++, i++ ) {

        // Add combination to feature vector
        features.at<float>(0,i) = rectangles.at(rect1).calculate_distance(rectangles.at(rect2));  
        
      }
    }

  } else {

    // Loop through required features
    for( std::vector<int>::iterator it = feature_ids.begin(); it != feature_ids.end(); it++) {

      // Calculate first rectangle in distance
      std::vector<int>::iterator offset = upper_bound(rect_partitions.begin(), rect_partitions.end(), *it); 

			int rect1 = offset - rect_partitions.begin(); 

			// Calculate second rectangle
			int rect2 = (*it - (*offset - *rect_partitions.begin())) + rect1 + 1;  
    
    	// Compute distance
			features.at<float>(0,*it) = rectangles.at(rect1).calculate_distance(rectangles.at(rect2));

		}

	}

}  
    

// Initialize rectangle to be used for calculating RDSF
RDSFRect::RDSFRect(cv::Rect target_rect, std::vector<cv::Mat> * input_integral_images) {

	// Initialize local variables
	rect = target_rect; 
	integral_images = input_integral_images; 
	valid = false;
	histograms = cv::Mat::zeros(1, DEPTH_BINS, CV_32FC1); 

}


// Computes the histogram for rectangle region selected
void RDSFRect::compute_histograms() {

	// Check if already computed	
	if( valid ) 
		return; 

	
	// Compute histograms from integral images
	for( int bin = 0; bin < DEPTH_BINS; bin++ ) {

		// This is dodgy but we're only using it locally - Pointer invalid if vector modified
		cv::Mat * integral_image = &((*integral_images).at(bin)); 

		// Calculate values in each bin
		histograms.at<float>(0, bin) = (*integral_image).at<float>(rect.br()) + (*integral_image).at<float>(rect.tl() - cv::Point(1,1)) - (*integral_image).at<float>(rect.x + rect.width-1, rect.y-1) - (*integral_image).at<float>(rect.x-1, rect.y + rect.height-1); 

	}

	// Normalize the histogram 
	cv::normalize(histograms, histograms, 1, 0, cv::NORM_L1); 

	// Set valid (histograms computed)
	valid = true;

}


// Calculates the Bhattacharyya distance between two histograms
float RDSFRect::calculate_distance(RDSFRect& rect2) {

	cv::Mat rect2_hist; 

	// Compute local histograms
	compute_histograms();

	// Get remote histograms
	rect2.get_histograms(rect2_hist); 

	float distance = 0;

	// Loop through and compute distance
	for( int bin = 0; bin < DEPTH_BINS; bin++ ) {

		distance += sqrt( histograms.at<float>(0,bin) * rect2_hist.at<float>(0,bin) ); 

	} 

	return distance; 

}


// Returns histograms of rectangle
void RDSFRect::get_histograms(cv::Mat& return_hist) {

	// Compute histograms
	compute_histograms(); 

	// Return
	return_hist = histograms;

}

		

	





  



