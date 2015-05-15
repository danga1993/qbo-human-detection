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
#include "performance/performance.h"


int main(int argc, char** argv)
{

	// Feature vector
	FeatureVector * features; 

	// Switch the feature vectors
	switch( FEATURE_VECTOR ) {
		case FEATURE_RDSF: 
			features = new RDSFVector(); 
			break; 
		case FEATURE_HOG: 
			features = new HOGVector(); 
			break; 
	}

	// Results
	int true_positives = 0; 
	int false_positives = 0; 
	int actual_positives = 0; 
	int actual_negatives = 0;


	// Measure performance
	if( TEST_SEGMENT_AUTO ) {

		Performance_Auto perform_auto(features, "boost.data"); 

		perform_auto.measure(true_positives, false_positives, actual_positives, "test_frames", TEST_MIS_DIR); 

	} else {

		Performance_Manual perform_manual(features, "boost.data");

		perform_manual.measure(true_positives, false_positives, actual_positives, actual_negatives, "test_data", TEST_MIS_DIR); 

	}

	// Compile statistics
	std::cout << "Precision: " << ((float)true_positives / (true_positives + false_positives));
	std::cout << " Recall: " << ((float)true_positives / actual_positives) << std::endl;

	std::cout << "True positives: " << true_positives; 
	std::cout << " False positives: " << false_positives; 
	std::cout << " Actual positives: " << actual_positives; 
	std::cout << " Actual negatives: " << actual_negatives; 
		
}



