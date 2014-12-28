class RDSFRect
{
	public:
	
		RDSFRect(cv::Rect target_rect, std::vector<cv::Mat> * input_integral_images); 
	 	void get_histograms(cv::Mat& return_hist); 
		float calculate_distance(RDSFRect& rect2); 

	private:

		cv::Rect rect; 
		cv::Mat histograms; 
		std::vector<cv::Mat> * integral_images; 
		int valid;

		void compute_histograms(); 

};



class RDSFVector : public FeatureVector
{
	public:
		RDSFVector();
		void set_candidate(const candidate& cand); 
		void getfeatures(std::vector<int> feature_ids, cv::Mat& features); 
		void getfeaturetype(int feature_id); 
		int getLength();

	private:
		std::vector<RDSFRect> rectangles; 
		std::vector<cv::Mat> integral_images;
		std::vector<int> rect_partitions;
		int length;

		
};

	

	

	
