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

};

		
	
