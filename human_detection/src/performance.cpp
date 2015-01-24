#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"
#include "tagger/tagger.h"
#include "config.h"

#include "feature/featurevector.h"
#include "feature/rdsfvector.h"
#include "feature/hogvector.h"

#include "segmenter/segmenter.h"


int main(int argc, char** argv)
{

	// Classifier
	CvBoost boost;

	// Feature vector
	FeatureVector * features; 

	// Results
	float true_positives = 0; 
	float false_positives = 0; 
	float actual_positives = 0; 

	// File for loading
	cv::FileStorage file;
	std::vector<std::string> files; 
	cv::Mat img; 

	std::vector<int> feature_ids;
	cv::Mat cand_features; 

	// Bounding boxes
	std::vector<cv::Rect> bounding_boxes; 
	std::vector<candidate> candidates; 

	// Initialize the classifier
	boost.load("boost.data"); 

	// Switch the feature vectors
	switch( FEATURE_VECTOR ) {
		case FEATURE_RDSF: 
			features = new RDSFVector(); 
			break; 
		case FEATURE_HOG: 
			features = new HOGVector(); 
			break; 
	}

	std::cout << "Loaded classifier and feature vectors" << std::endl;

	// Retrieve test data frames 
	if( TEST_TAG_AUTO ) {

		// Get list of files
		directory_list(files, "test_data"); 

		std::cout << "Found " << files.size() << " files" << std::endl;

		// Loop through 
		for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

			file.open(*it, cv::FileStorage::READ); 
			if( !file.isOpened() )
				{ std::cout << "Failed to open file " << *it << std::endl; exit(1); }

			// Load the frame data
			file["puka"] >> img; 
			file["boundingbox"] >> bounding_boxes; 

			// Resegment several times as segmentation has random element
			for( int seg_avg = 0; seg_avg < AVERAGE_SEGMENT_COUNT; seg_avg++ ) {

				// Segment the frame
				Segmenter::segment(img, candidates);

			 	for (std::vector<candidate>::iterator it_cand = candidates.begin(); it_cand != candidates.end(); it_cand++) {

					if( it_cand->erased ) 
						continue; 

					// Extract features
					features->set_candidate(*it_cand);  
					features->getfeatures(feature_ids, cand_features);

					//std::cout << "Prediction: " <<  boost.predict(cand_features) << std::endl;

					float prediction = boost.predict(cand_features, cv::Mat(), cv::Range::all(), false, true); 

					// Run each candidate through classifier
					if( prediction > BOOST_THRESHOLD ) {
							it_cand->classification = true; 
					} else
							it_cand->classification = false;

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

	} else if ( !TEST_TAG_AUTO ) {

				// loop through candidates

				// Extract features

				// Run candidate through classifier

				// Classify and store correct/not correct

	}

	// Compile statistics
	std::cout << "Precision: " << (true_positives / (true_positives + false_positives));
	std::cout << " Recall: " << (true_positives / actual_positives) << std::endl;

	std::cout << "True positives: " << true_positives; 
	std::cout << " False positives: " << false_positives; 
	std::cout << " Actual positives: " << actual_positives; 
		
}



