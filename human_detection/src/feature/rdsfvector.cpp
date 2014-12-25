#include <vector>
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.h"

void RDSFVector::set_candidate(const candidate& cand) {

	std::vector<cv::Mat> hist_images(DEPTH_BINS+1); 
	std::vector<cv::Mat> integral_images(DEPTH_BINS);
	cv::Size imsize(cand->im.size()); 

	cv::Mat cand_norm;

	// Normalise candidate image
	cv::normalize(cand->im, cand_norm, 0, 1, cv::NORM_MINMAX);

	// Fill first bin with 1's (because threshold is at 0) 
	hist_images.push_back(ones(imsize, CV_32FC1));

	// Separate image into histogram bins and calculate integral images for each bin
	for( int bin = 0; bin < DEPTH_BINS; bin++ ) {

		hist_images.emplace_back(); 

		if( bin < DEPTH_BINS-1 ) {
			// Threshold the depth image
			cv::threshold(cand->im, hist_images.at(bin+1), (bin+1)/DEPTH_BINS, 1.0, cv::THRESH_BINARY); 
		} else {
			// Last thresholded image all zeros
			hist_images.at(bin+1) = zeros(imsize, CV_32FC1); 
		}

		// We might not need this if floats subtract to zero as required
		// hist_images[bin+1].convertTo(hist_images[bin+1], CV_8UC1);

		// Calculate histogram bin image (difference of thresholded images)
		hist_images.at(bin) = hist_images.at(bin) - hist_images.at(bin+1); 

		// Calculate integral image
		cv::integral(hist_images.at(bin), integral_images.at(bin)); 	

	}

} 



