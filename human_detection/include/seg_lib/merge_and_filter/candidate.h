#include "config.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <algorithm>    // std::min
#include <vector>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#ifndef CANDIDATE_H
#define CANDIDATE_H


class candidate {
	public:
		//descriptor requirements
		cv::Rect boundingBox;
		cv::Mat im;
		int classification;

		//TODO
		bool erased;
		bool merged;
		bool human;

		//candidate features for quick heuristic rejection
		float max_inlier_fraction;
		float real_height;
		float real_width;
		cv::Point3f centre;

		//variables to help calculate candidate features
		int xmin, xmax, ymin, ymax;
		float depth_accumulator;

		//variables for other aids
		std::vector<cv::Point3f> pts;
		int id;

		//Constructors & destructors
		candidate();
		//candidate(int s);
		candidate(cv::Mat& input_im, bool input_human);
		candidate(int x,int y, float z, int i);
		~candidate();

		//member functions
		void add(int x, int y, float z);
		void set_boundingBox();
		void create_candidate_image(cv::Mat &depthim);
		bool merge(candidate c);
		void calc_centre();
		int size() const;
		void RANSAC_inliers();
		void calc_real_dims();
		float calc_real_distance(float depth, int p, int length, float fov);
		//void add_to_image(image<rgb> * im, rgb colour);
};


#endif
