class RDSFVector
{
	public:
		void RDSFVector();
		void set_candidate(const candidate& cand); 
		void getfeatures(int[] feature_ids, cv::Mat& features); 
		void getfeaturetype(int feature_id); 
		int length; 

	private:
		std::vector<RDSFRect> rectangles; 
		std::vector<cv::Mat> integral_images;

		
};


class RDSFRect
{
	public:
	
		void RDSFRect(cv::Rect& target_rect, std::vector<cv::Mat> * input_integral_images); 
	  void get_histograms(cv::Mat& return_hist); 
		void calculate_distance(RDSFRect& rect); 

	private:

		cv::Rect rect; 
		cv::Mat histograms; 
		std::vector<cv::Mat> * integral_images; 
		int valid; 

		void compute_histograms(); 

};


	

	

	
