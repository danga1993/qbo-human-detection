#include <vector>
#include <time.h>

#include "segmenter.h"
#include "../seg_lib/segment_depth/segment.h"

Segmenter::segment(image<float>* img)
{
	// parameters
	const float sigma = SIGMA;
	const float kdepth = KDEPTH;
	const float knormal = KNORMAL;
	const int min_sizr = MIN_SIZE;
	const int alpha = ALPHA;
	const int s = ESS;

	// conversion parameters
	int in_width = img.size().width;
	int in_height = img.size().height;
	int sub_width = in_width / alpha;
	int sub_height = in_height / alpha;

	// seed for random generator
	srand(time(NULL));

	// subsample image
	float lastgoodvalue = 0.0f;
	float dval = 0.0f;
	image<float> sub_img = new image<float>(sub_width, sub_height);

	for (int y = 0; y < sub_height; y++) {
		for (int x = 0; x < sub_width; x++) {
			std::vector<float> samples;
			for (int i = 0; i <= s; i++)
			{
				int dx = rand() % alpha;
				int dy = rand() % alpha;

				dval = img.at<float>(y * alpha + dy, x * alpha + dx);
				if (!isnan(dval))
					samples.push_back(dval);
			}

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
}
