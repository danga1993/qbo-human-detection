#include "seg_lib/merge_and_filter/candidate.h"

class FeatureVector
{
	public:
		virtual void getfeatures(std::vector<int> feature_ids, cv::Mat& features) = 0; 
		virtual void set_candidate(const candidate cand) = 0; 
		virtual void getfeaturetype(int feature_id) = 0;
		virtual int getLength() = 0;
};
