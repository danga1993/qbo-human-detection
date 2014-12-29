#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "feature/featurevector.h"
#include "feature/hogvector.h"
#include "config.h"

void HOGVector::set_candidate(const candidate& cand)
{
	cv::Size cand_size(CANDIDATE_WIDTH, CANDIDATE_HEIGHT);

	int cell_cols = CANDIDATE_WIDTH / HOG_CELL_SIZE; 
	int cell_rows = CANDIDATE_HEIGHT / HOG_CELL_SIZE; 
	int cell_count = cell_cols * cell_rows; 

	cv::Mat diff_hor; 
	cv::Mat diff_vert; 

	// Initialize cell vector
	cells = std::vector< std::vector<cv::Mat> >(cell_rows, std::vector<cv::Mat>(cell_cols));  

	// Kernels for differentiation [-0.5 0 0.5] and [-0.5 0 0.5]t
	cv::Mat dx_kern = (cv::Mat_<float>(1,3) << -0.5, 0, 0.5); 
	cv::Mat dy_kern = (cv::Mat_<float>(3,1) << -0.5, 0, 0.5);
	cv::Point anchor(-1, -1); 

	// Differentiate image
	cv::filter2D(cand.im, diff_hor, CV_32F, dx_kern, anchor, 0, cv::BORDER_REPLICATE);
	cv::filter2D(cand.im, diff_vert, CV_32F, dy_kern, anchor, 0, cv::BORDER_REPLICATE);

	// Calculate angle of each pixel (uses fast atan2)
	cv::cartToPolar(diff_hor, diff_vert, polar_mag, polar_angle, true); 

	// Length of feature vector - CHECK
	length = HOG_BIN_COUNT * HOG_BLOCK_SIZE * HOG_BLOCK_SIZE * ((cell_cols / HOG_BLOCK_SHIFT) - ((HOG_BLOCK_SIZE - (cell_cols % HOG_BLOCK_SHIFT) - HOG_BLOCK_SHIFT) / HOG_BLOCK_SHIFT)) * ((cell_rows / HOG_BLOCK_SHIFT) - ((HOG_BLOCK_SIZE - (cell_rows % HOG_BLOCK_SHIFT) - HOG_BLOCK_SHIFT) / HOG_BLOCK_SHIFT));

}

// Calculates histogram for each cell in image
void HOGVector::compute_cells() {

	// Loop through and produce histogram of cells
	for( int y = 0, cell_y = 0; y < CANDIDATE_HEIGHT; y += HOG_CELL_SIZE, cell_y++ ) {
		for( int x = 0, cell_x = 0; x < CANDIDATE_WIDTH; x += HOG_CELL_SIZE, cell_x++ ) {

			// Initialize cell histogram
			cv::Mat histogram = cv::Mat::zeros(1, HOG_BIN_COUNT, CV_32F);

			// Loop through cell pixels
			for( int offset_y = 0; offset_y < HOG_CELL_SIZE; offset_y++ ) {
				for( int offset_x = 0; offset_x < HOG_CELL_SIZE; offset_x++ ) {

					// Split the value between the two nearest bins
					float bin_exact = polar_angle.at<float>(y + offset_y, x + offset_x) * HOG_BIN_COUNT / 360; 
					int bin_floor = cvFloor(bin_exact); 
					int bin_ceil  = cvCeil(bin_exact); 

					// Weight according to distance from bin
					float weight_floor = bin_ceil - bin_exact;
					float weight_ceil = 1 - weight_floor; 

					// Make sure highest bin (360) wraps around to (0)
					bin_ceil = (bin_ceil >= HOG_BIN_COUNT) ? 0 : bin_ceil; 

					float mag = polar_mag.at<float>(y + offset_y, x + offset_x);

					if( mag > 0 ) {
					
						histogram.at<float>(bin_floor) += weight_floor * mag; 
						histogram.at<float>(bin_ceil) += weight_ceil * mag; 

					}

				}
			}

			// Add to array of cells
			cells.at(cell_y).at(cell_x) = histogram; 

		}
	}

}


// Computes histogram blocks of several cells and adds to descriptor
void HOGVector::compute_descriptor(cv::Mat& descriptor) {

	int cell_cols = CANDIDATE_WIDTH / HOG_CELL_SIZE; 
	int cell_rows = CANDIDATE_HEIGHT / HOG_CELL_SIZE;

	// Clear matrix
	descriptor = cv::Mat::zeros(0, 0, CV_32F); 

	// NOTE: Previous program uses HOD_BLOCK_SHIFT - Possible error
	for( int y = 0; y < cell_rows - (HOG_BLOCK_SIZE-1); y += HOG_BLOCK_SHIFT ) {
		for( int x = 0; x < cell_cols - (HOG_BLOCK_SIZE-1); x += HOG_BLOCK_SHIFT ) {

			cv::Mat block; 

			// Get histograms for block
			for( int cell_y = 0; cell_y < HOG_BLOCK_SIZE; cell_y++ ) {
				for( int cell_x = 0; cell_x < HOG_BLOCK_SIZE; cell_x++ ) {

					if( !block.empty() ) {
						cv::hconcat(block, cells.at(y + cell_y).at(x + cell_x), block); 
					} else {
						block = cells.at(y + cell_y).at(x + cell_x).clone();
					}

				}
			}

			// Normalize block
			cv::normalize(block, block, 1, 0, cv::NORM_L2, CV_32F); 

			// Add to descriptor
			if( !descriptor.empty() ) {
				cv::hconcat(descriptor, block, descriptor); 
			} else {
				descriptor = block; 
			}

		}
	}

}
		
			
// Retrieves vector of features (NOTE: feature_ids ignored at the moment)
void HOGVector::getfeatures(std::vector<int> feature_ids, cv::Mat& features)
{
	
	compute_cells(); 
	compute_descriptor(features); 

}

int HOGVector::getLength() {
	return length; 
}


HOGVector::HOGVector() {
}

void HOGVector::getfeaturetype(int feature_id) {

}
