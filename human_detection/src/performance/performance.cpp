#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"
#include "tagger/tagger.h"
#include "config.h"

#include "feature/featurevector.h"
#include "segmenter/segmenter.h"
#include "performance/performance.h"


// Initialize the performance class 
Performance::Performance(FeatureVector * features_input, std::string boost_data) {

	// Initialize the classifier
	boost.load("boost.data"); 

	// Store the feature vector
	features = features_input; 

	std::cout << "Loaded classifier and feature vectors" << std::endl;

}

		
// Pass candidate to the classifier
void Performance::classify_candidate(candidate& cand) {
	
	if( cand.erased ) {
		std::cout << "Classifying an erased candidate" << std::endl; 
		return; 
	}
	
	// Feature extraction
	std::vector<int> feature_ids;
	cv::Mat cand_features;  

	// Extract features
	features->set_candidate(cand);  
	features->getfeatures(feature_ids, cand_features);

	//std::cout << "Prediction: " <<  boost.predict(cand_features) << std::endl;

	float prediction = boost.predict(cand_features, cv::Mat(), cv::Range::all(), false, true); 

	// Run each candidate through classifier
	if( prediction > BOOST_THRESHOLD ) {
			cand.classification = true; 
	} else
			cand.classification = false;

}

// Constructur calls parent constructor
Performance_Auto::Performance_Auto(FeatureVector * features_input, std::string boost_data) :
	Performance(features_input, boost_data) { }

	
// Perform automatic segmentation and tagging using bounding boxes
void Performance_Auto::measure(int& true_positives, int& false_positives, int& actual_positives, std::string frame_dir) {
		
		// Bounding boxes
		std::vector<cv::Rect> bounding_boxes; 
		std::vector<candidate> candidates; 

		// File for loading
		std::vector<std::string> files; 
		cv::FileStorage file;

		// Frame image
		cv::Mat img; 
		
		// Get list of files
		directory_list(files, frame_dir); 

		std::cout << "Found " << files.size() << " files" << std::endl;

		// Loop through 
		for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

			file.open(*it, cv::FileStorage::READ); 
			if( !file.isOpened() )
				{ std::cout << "Failed to open file " << *it << std::endl; exit(1); }

			// Load the frame data
			file["puka"] >> img; 
			file["bounding_positive"] >> bounding_boxes; 

			// Resegment several times as segmentation has random element
			for( int seg_avg = 0; seg_avg < AVERAGE_SEGMENT_COUNT; seg_avg++ ) {

				// Segment the frame
				Segmenter_Auto::segment(img, candidates);

			 	for (std::vector<candidate>::iterator it_cand = candidates.begin(); it_cand != candidates.end(); it_cand++) {

					classify_candidate(*it_cand); 

				}

				// Now match with boxes
				for( std::vector<cv::Rect>::iterator box = bounding_boxes.begin(); box != bounding_boxes.end(); box++ ) {

					std::cout << "Bounding box" << *box << std::endl;

					int box_match = 0; 

					for (std::vector<candidate>::iterator it_cand = candidates.begin(); it_cand != candidates.end(); it_cand++) {

						if( it_cand->erased ) 
							continue; 

						// Check if box matched
						if( Tagger::candidate_intersect(*it_cand, *box) ) {
		
							it_cand->human = true; 
						
							// A positive candidate matches the box
							if( it_cand->classification ) 
								box_match++; 

						}
						
					}

					if( box_match > 1 ) {
						std::cout << "More than one candidate per box" << std::endl;
					}
						
					// The box has been matched by at least one candidate
					if( box_match > 0) {
						true_positives++; 
					}

					if( box_match == 0 ) 	
						std::cout << "Not Matched" << std::endl;

				}

				// Find false positives
		 		for (std::vector<candidate>::iterator it_cand = candidates.begin(); it_cand != candidates.end(); it_cand++) {

					if( it_cand->erased ) 
					continue; 
			
					if( it_cand->classification == true && it_cand->human == false ) 
						false_positives++;

				}

				actual_positives += bounding_boxes.size();

			}

		}

}

// Constructur calls parent constructor
Performance_Manual::Performance_Manual(FeatureVector * features_input, std::string boost_data) :
	Performance(features_input, boost_data) { }

// Asses performance with manually segmented candidate
void Performance_Manual::measure(int& true_positives, int& false_positives, int& actual_positives, int& actual_negatives, std::string candidate_dir) {

	// File for loading
	std::vector<std::string> files; 
	cv::FileStorage file;

	// Candidate data
	cv::Mat img; 
	int human; 
	cv::Mat cand_features; 

	// Features to extract from candidate
	std::vector<int> feature_ids;	
	
	// Get list of files
	directory_list(files, candidate_dir); 

	std::cout << "Found " << files.size() << " files" << std::endl;

	// Loop through 
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

	  file.open(*it, cv::FileStorage::READ); 
		if( !file.isOpened() )
			{ std::cout << "Failed to open file " << *it << std::endl; exit(1); }

		// Read data out of candidate
		file["image"] >> img; 
		file["human"] >> human;

		file.release();

		// Generate candidate from stored file (Note: candidate missing many fields)
		candidate cand(img, human); 

		// Classify candidate
		classify_candidate(cand); 

		// Add classification to statistics
		if( cand.human ) {
			actual_positives++; 

			if( cand.classification ) 
				true_positives++; 
		
		} else {
			actual_negatives++; 

			if( cand.classification ) 
				false_positives++; 

		}

	}
				
}









