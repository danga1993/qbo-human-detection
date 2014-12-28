class HOGVector extends FeatureVector
{
	public:
		void HOGVector();
		void set_candidate(const candidate& cand);
		void getfeatures(std::vector<int> feature_ids, cv::Mat& features);
		void getfeaturetype(int feature_id);
}