#include "seg_lib/merge_and_filter/candidate.h"

class Segmenter
{

};

class Segmenter_Auto : public Segmenter 
{ 
	public: 
		static void segment(cv::Mat& img, std::vector<candidate>& candidates);
	
	private:
		static image<float> * subsample(cv::Mat& img);

};


class Segmenter_Manual : public Segmenter 
{ 
	public: 
		static void segment(cv::Mat& img, std::vector<candidate>& candidates, std::vector<cv::Rect> bounding_positive, std::vector<cv::Rect> bounding_negative);

	private: 
		static void create_candidate_image(cv::Mat& depth_img, cv::Mat& cand_img, cv::Rect& region);

};

		
	
