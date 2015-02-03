class Performance
{
	public:
		
		Performance(FeatureVector * features_input, std::string boost_data); 
		void measure_auto(int& true_positives, int& false_positives, int& actual_positives);
		
	protected: 

		void classify_candidate(candidate& cand);

	private: 

		FeatureVector * features;
		CvBoost boost; 

};


class Performance_Manual : public Performance
{
	public: 
		Performance_Manual(FeatureVector * features_input, std::string boost_data);
		void measure(int& true_positives, int& false_positives, int& actual_positives, int& actual_negatives, std::string candidate_dir);

};

class Performance_Auto : public Performance
{
	public: 
		Performance_Auto(FeatureVector * features_input, std::string boost_data);
		void measure(int& true_positives, int& false_positives, int& actual_positives, std::string frame_dir);

};
