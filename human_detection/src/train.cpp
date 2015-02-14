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


// Loads features from given file and adds to training vector
void load_features(std::string path, int label, FeatureVector * features, std::vector<int> feature_ids, cv::Mat& train_features, cv::Mat& train_labels) {

		cv::Mat img; 
		cv::Mat cand_features; 

		// Load depth image
		image_read(path + "/depth.png", img); 

		std::cout << "Loading " << path << std::endl;

		// Generate candidate from stored file (Note: candidate missing many fields)
		candidate cand(img, label); 

		// Pass candidate to feature extractor
		features->set_candidate(cand);  

		// Extract the features
		features->getfeatures(feature_ids, cand_features);
	
		// Add onto the main vector
		train_features.push_back(cand_features);

		// Add to labels
		train_labels.push_back((float)label); 

}


int main(int argc, char** argv)
{

	// File for loading
	std::vector<std::string> files; 

	FeatureVector * features; 

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
	directory_list(files, "train_data/positive"); 

	// Loop through 
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

		// Load the features from candidate into the training vector
		load_features(*it, 1, features, feature_ids, train_features, train_labels); 

	}

	// Get list of files
	directory_list(files, "train_data/negative"); 

	// Loop through 
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

		// Load the features from candidate into the training vector
		load_features(*it, 0, features, feature_ids, train_features, train_labels); 

	}

	/*cv::Mat temp = cv::Mat::zeros(1, 120786, CV_32F);  
	train_features.push_back(temp); 
	train_labels.push_back((float)0); 

	temp = cv::Mat::ones(1, 120786, CV_32F);  
	train_features.push_back(temp); 
	train_labels.push_back((float)1); 
	
	for( int i = 0; i < 10; i++ ) {
		train_features.push_back(train_features.row(i)); 
		train_labels.push_back(train_labels.row(i)); 
	} 

	cv::Mat row0 = train_features.row(0); 
	cv::Mat row1 = train_features.row(1); */

	//std::cout << "Equ: " << (row0 == row1) << std::endl;
		

	std::cout << "Train vector: " << train_features.size() << std::endl;
	std::cout << "Label vector: " << train_labels << std::endl;

	// Send to classifier - NOTE: TRY REAL ADABOOST
	CvBoostParams params(CvBoost::REAL, 100, 0, 1, false, NULL); 

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



