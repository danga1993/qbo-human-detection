class Labeller {

	public: 

		Labeller();
    void set_frame(cv::Mat& in_img);
    void label_frame(); 
    void get_labels(std::vector<cv::Rect>& label_out_positive, std::vector<cv::Rect>& label_out_negative); 

  private: 

    int keyprocess(char c); 
    static void mouseprocess(int event, int x, int y, int flag, void * instance); 
  
    std::vector<cv::Rect> labels_positive;
		std::vector<cv::Rect> labels_negative;
    cv::Rect active_label; 
    cv::Mat img; 
    cv::Mat img_draw; 
    bool display_active; 

};
