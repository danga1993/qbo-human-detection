#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"

#include "feature/featurevector.h"
#include "feature/rdsfvector.h"

#include "config.h"


int main(int argc, char** argv)
{

	// File for loading
	cv::FileStorage file;
	std::vector<std::string> files; 

	FeatureVector * features; 
	cv::Mat cand_features; 
	cv::Mat train_features; 

	cv::Mat img; 
	int human; 

	std::vector<int> feature_ids;

	// Switch the feature vectors
	if( FEATURE_VECTOR == "rdsf" ) {
		features = new RDSFVector();
	}

	// Initialise matrix columns
	train_features.create(0, features->getLength(), CV_32FC1);

	// Get list of files
	directory_list(files, "train_data"); 

	// Loop through 
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

		file.open(*it, cv::FileStorage::READ); 
		if( !file.isOpened() )
			{ std::cout << "Failed to open file " << *it << std::endl; exit(1); }

		// Read data out of candidate
		file["image"] >> img; 
		file["human"] >> human;

		file.release();

		// Generate candidate from stored file
		candidate cand(img, human); 

		// Pass candidate to feature extractor
		features->set_candidate(cand);  

		// Extract the features
		features->getfeatures(feature_ids, cand_features);
	
		// Add onto the main vector
		train_features.push_back(cand_features);

	}
	
}



