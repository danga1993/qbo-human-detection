
#include "seg_lib/segment_depth/segment.h"
#include "seg_lib/merge_and_filter/candidate.h"

// Displays an image using OpenCV and waits for user
void displayImg(cv::Mat& img, std::string window_name = ""); 

// Displays an image of class image (Felzenswalb)
void display_felzen(image<rgb> * img);
void display_felzen(image<float> * img);

// Lists files in a directory
void directory_list(std::vector<std::string>& files, std::string path); 

// Generates an image to visualize segmented candidates
void display_candidates(int width, int height, std::vector<candidate>& candidates);
  

	
