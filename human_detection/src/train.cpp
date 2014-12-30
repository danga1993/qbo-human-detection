#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

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
	cv::Mat train_labels; 

	// Classifier
	CvBoost boost;
	
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

		// Add to labels
		train_labels.push_back((float)cand.human); 

		/*std::cout << "Actual: " << train_features.size() << std::endl;
		std::cout << "Theory: " << features->getLength() << std::endl;*/

	}

	std::cout << "Train vector: " << train_features.size() << std::endl;
	std::cout << "Label vector: " << train_labels.size() << std::endl;

	// Send to classifier - NOTE: TRY REAL ADABOOST
	CvBoostParams params(CvBoost::REAL, 10, 0, 1, false, NULL); 

	// Create masks
	cv::Mat sample_idx = cv::Mat::ones(1, train_features.rows, CV_8U); 
	cv::Mat var_type( train_features.cols + 1, 1, CV_8U ); 
	
	var_type.setTo(cv::Scalar::all(CV_VAR_ORDERED)); 
	var_type.at<uchar>( train_features.cols ) = CV_VAR_CATEGORICAL; 
	
	// Train
	if( !boost.train(train_features, CV_ROW_SAMPLE, train_labels, cv::Mat(), sample_idx, var_type, cv::Mat(), params, false) ) 
		std::cout << "Boost error"; 

	// Save the trained classifier
	boost.save("boost.data"); 
	
}



