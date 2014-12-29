class HOGVector : public FeatureVector
{
	public:
		HOGVector();
		void set_candidate(const candidate& cand);
		void getfeatures(std::vector<int> feature_ids, cv::Mat& features);
		void getfeaturetype(int feature_id);
		int getLength();

		void compute_cells(); 
		void compute_descriptor(cv::Mat& descriptor);

	private: 
		
		std::vector< std::vector<cv::Mat> > cells; 
		cv::Mat polar_mag; 
		cv::Mat polar_angle; 
		int length;
};
