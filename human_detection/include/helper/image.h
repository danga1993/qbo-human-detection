#include "seg_lib/segment_depth/segment.h"

// Displays an image using OpenCV and waits for user
void displayImg(cv::Mat& img, std::string window_name = ""); 

// Displays an image of class image (Felzenswalb)
void display_felzen(image<rgb> * img);

// Stores a Mat file image as a PNG
void image_write(std::string filename, cv::Mat& img); 

// Reads into a Mat file from PNG image
void image_read(std::string filename, cv::Mat& img);

// Lists files in a directory
void directory_list(std::vector<std::string>& files, std::string path); 

// Make a directory if it does not exist
void make_directory(std::string path);
  

	
