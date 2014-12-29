#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"

#include "feature/featurevector.h"
#include "feature/rdsfvector.h"
#include "feature/hogvector.h"

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
	switch( FEATURE_VECTOR ) {
		case FEATURE_RDSF: 
			features = new RDSFVector(); 
			break; 
		case FEATURE_HOG: 
			features = new HOGVector(); 
			break; 
	}

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

		// Generate candidate from stored file (Note: candidate missing many fields)
		candidate cand(img, human); 

		// Pass candidate to feature extractor
		features->set_candidate(cand);  

		// Extract the features
		features->getfeatures(feature_ids, cand_features);
	
		// Add onto the main vector
		train_features.push_back(cand_features);

		std::cout << "Actual: " << train_features.size() << std::endl;
		std::cout << "Theory: " << features->getLength() << std::endl;

	}
	
}



