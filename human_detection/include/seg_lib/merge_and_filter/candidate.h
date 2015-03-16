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
		int edge_count;

		//variables for other aids
		std::vector<cv::Point3f> pts;
		int id;

		// Edges
		std::map<int,int> edges;

		//Constructors & destructors
		candidate();
		//candidate(int s);
		candidate(cv::Mat& input_im, bool input_human);
		candidate(int x,int y, float z, int i);
		~candidate();

		//member functions
		void add(int x, int y, float z);
		void add_edge(int vertex_id);
		void set_boundingBox();
		void create_candidate_image(cv::Mat &depthim);
		void create_candidate_image(cv::Mat &depthim, cv::Rect& region);
		void position_candidate_image(cv::Mat& cand_window, cv::Mat& cand_image); 
		bool merge(candidate c);
		void calc_centre();
		int size() const;
		void RANSAC_inliers();
		void calc_real_dims();
		void calc_real_coords(float depth, int x, int y, float& real_x, float& real_y);
		//void add_to_image(image<rgb> * im, rgb colour);
};


#endif
