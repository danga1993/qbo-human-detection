#include <vector>
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.h"

#include "seg_lib/segment_depth/segment-image.h"
#include "seg_lib/merge_and_filter/merge_and_filter.h"

#include "helper/image.h"
#include "segmenter/segmenter.h"

// Manually segments a frame using the tagged boxes
void Segmenter_Manual::segment(cv::Mat& img, std::vector<candidate>& candidates, std::vector<cv::Rect> bounding_positive, std::vector<cv::Rect> bounding_negative) {

	// Clean the candidate vector
	candidates.clear(); 

	// Candidate id
	int i = 0; 

	// Loop through positives
	for (std::vector<cv::Rect>::iterator it = bounding_positive.begin(); it != bounding_positive.end(); it++, i++) {
	
		std::cout << "Creating candidate" << std::endl;

		// Create candidate
		candidate cand(it->x/ALPHA, it->y/ALPHA, 0, i);
		create_candidate_image(img, cand.im, *it); 
		cand.human = true; 
		candidates.push_back(cand); 

	}

	// Loop through negatives
	for (std::vector<cv::Rect>::iterator it = bounding_negative.begin(); it != bounding_negative.end(); it++, i++) {

		// Create candidate
		candidate cand(it->x/ALPHA, it->y/ALPHA, 0, i);
		create_candidate_image(img, cand.im, *it); 
		cand.human = false; 
		candidates.push_back(cand); 

	}

}


// Scales a rectangular region to fit candidate window
void Segmenter_Manual::create_candidate_image(cv::Mat& depth_img, cv::Mat& cand_img, cv::Rect& region) {

		// Generate candidate image (NOTE: Using section from built in create_candidate_image function)
		float scaling = std::min((float)CANDIDATE_WIDTH/(region.width), (float)CANDIDATE_HEIGHT/(region.height));
	
		// Scaled dimensions
		cv::Size dims = cv::Size(floor(region.width * scaling), floor(region.height * scaling)); 
		
		//std::cout << "Scaled candidate - width: " << dims.width << " height: " << dims.height << std::endl;

		cand_img = cv::Mat::zeros(cv::Size(CANDIDATE_WIDTH, CANDIDATE_HEIGHT), CV_32FC1);

		cv::Point image_centre = cv::Point((CANDIDATE_WIDTH)/2, (CANDIDATE_HEIGHT)/2);
		cv::Point start = image_centre - cv::Point(dims.width/2, dims.height/2);

		//std::cout << "Centre: (" << image_centre.x << "," << image_centre.y << ")" << std::endl;
		//std::cout << "Start: (" << start.x << "," << start.y << ")" << std::endl;

		std::cout << "Filling candidate window" << std::endl;

		// Center the scaled candidate in the candidate window
		for(int x = 0; x < dims.width; x++){
			for(int y = 0; y < dims.height; y++){

				cv::Point src(region.x + x/scaling, region.y + y/scaling); 
				cv::Point dest(start+cv::Point(x,y)); 

					std::cout << "Copying (" << src.x << "," << src.y << ") to (" << dest.x << "," << dest.y << ")" << std::endl;

				cand_img.at<float>(start+cv::Point(x,y)) = depth_img.at<float>(cv::Point(region.x + x/scaling, region.y + y/scaling));

			
			}
		}

		std::cout << "Filled" << std::endl;

}

// Segments the image using graph-based segmentation algorithm
void Segmenter_Auto::segment(cv::Mat& img, std::vector<candidate>& candidates)
{
	// parameters
	float sigma = SIGMA;
	float kdepth = KDEPTH;
	float knormal = KNORMAL;
	int min_size = MIN_SIZE;
	int num_ccs = 0;


	// Clean the candidate vector
	candidates.clear(); 

	// subsample image
	image<float> * sub_img = subsample(img);

	// segment
	image<rgb>* normal_img;
	image<rgb>* depthseg;
	image<rgb>* normalseg;
	image<rgb>* jointseg;

	universe* u_segmented = segment_image1C(sub_img, sigma, kdepth, knormal, min_size, &num_ccs, &normal_img, &depthseg, &normalseg, &jointseg);

	//display_felzen(jointseg); 

	// merge regions
	merge_and_filter(sub_img, u_segmented, sub_img->width(), sub_img->height(), img, candidates);

	// Free all dynamic memory (not great C++)
	delete normal_img;
	delete depthseg; 
	delete normalseg; 
	delete jointseg;
	delete sub_img; 
	delete u_segmented; 

}


// Subsamples image before segmenting to reduce graph size
image<float> * Segmenter_Auto::subsample(cv::Mat& img)
{
	// fixed parameters
	const int alpha = ALPHA;
	const int s = ESS;

	// conversion parameters
	int in_width = img.size().width;
	int in_height = img.size().height;
	int sub_width = in_width / alpha;
	int sub_height = in_height / alpha;

	// seed for random generator
	srand(time(NULL));

	float lastgoodvalue = 0.0f;
	float dval = 0.0f;

	// Container for new subsampled image
	image<float> * sub_img = new image<float>(sub_width, sub_height);

	// For each pixel of the subsampled image
	for (int y = 0; y < sub_height; y++) {
		for (int x = 0; x < sub_width; x++) {

			std::vector<float> samples;

			// Randomly sample pixels from corresponding alphaxalpha region of original image
			for (int i = 0; i <= s; i++)
			{
				int dx = rand() % alpha;
				int dy = rand() % alpha;

				dval = img.at<float>(y * alpha + dy, x * alpha + dx);

				if (!isnan(dval))
					samples.push_back(dval);
			
			}

			// Pixels in region all NaN (kinect shadow) - Use value from previous region (Match the background)
			if (samples.empty())
			{
				sub_img->access[y][x] = lastgoodvalue;
			}
			else
			{
				// arrange samples with median at index s/2
				std::nth_element(samples.begin(), samples.begin() + samples.size() / 2, samples.end());

				// take median to be the sample point
				sub_img->access[y][x] = samples[samples.size() / 2];
				lastgoodvalue = samples[samples.size() / 2];
			}
		}
	}

	return sub_img; 


}
