#include "seg_lib/merge_and_filter/candidate.h"

// Segmenter class declaration 
class Tagger
{
	public:
		static void tag(std::vector<candidate>& candidates); 
		static void tag(std::vector<candidate>& candidates, std::vector<cv::Rect>& bounding_boxes);
	
};

