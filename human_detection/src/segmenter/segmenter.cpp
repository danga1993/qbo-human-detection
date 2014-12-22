#include <vector>
#include <time.h>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "config.h"

#include "seg_lib/segment_depth/segment-image.h"
#include "seg_lib/merge_and_filter/merge_and_filter.h"

#include "helper/image.h"

// Segmenter class declaration 
class Segmenter
{
	public:
		static void segment(cv::Mat& img, std::vector<candidate>& candidates);

	private:
		static image<float> * subsample(cv::Mat& img);

};


// Segments the image using graph-based segmentation algorithm
void Segmenter::segment(cv::Mat& img, std::vector<candidate>& candidates)
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
	image<float> * sub_img = Segmenter::subsample(img);

	// segment
	image<rgb>* normal_img;
	image<rgb>* depthseg;
	image<rgb>* normalseg;
	image<rgb>* jointseg;

	universe* u_segmented = segment_image1C(sub_img, sigma, kdepth, knormal, min_size, &num_ccs, &normal_img, &depthseg, &normalseg, &jointseg);

	display_felzen(jointseg); 

	// merge regions
	merge_and_filter(sub_img, u_segmented, sub_img->width(), sub_img->height(), img, candidates);

	// Free all dynamic memory (this is awful C++)
	delete normal_img;
	delete depthseg; 
	delete normalseg; 
	delete jointseg;
	delete sub_img; 
	delete u_segmented; 

}


// Subsamples image before segmenting to reduce graph size
image<float> * Segmenter::subsample(cv::Mat& img)
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
