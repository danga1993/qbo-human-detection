#include <sstream>

#include <iostream>
#include <dirent.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"

// Displays an image using OpenCV and waits for user
void displayImg(cv::Mat& img)
{

	static int win_num = 0; 
	std::ostringstream win_name; 

	// scale values to 8-bit
	cv::Mat img_disp = img.clone();
	double minval, maxval;
	cv::minMaxIdx(img_disp, &minval, &maxval);

	img_disp.convertTo(img_disp, CV_8UC1, 255.0/maxval);

	win_name << "PUKA" << win_num;

	// show image
	cv::namedWindow(win_name.str(), CV_WINDOW_NORMAL);
	cv::imshow(win_name.str(), img_disp);

	win_num++; 

}


// Display a Felzenimage
void display_felzen(image<rgb> * img) {

  cv::Mat img_disp(img->height(), img->width(), CV_32FC1); 

	// Loop through and convert to matrix
  int img_height = img->height(); 
  int img_width = img->width(); 

  for( int y = 0; y < img_height; y++ ) {
    for( int x = 0; x < img_width; x++ ) {

      img_disp.at<float>(y, x) = (float)imRef(img, x, y).r; 

    }
  }

  // Display
  displayImg(img_disp); 

}


// List the files in a directory
void directory_list(std::vector<std::string>& files, std::string path) {

	struct dirent *entry; 
	DIR *dp; 

	dp = opendir(path.c_str()); 

	if( dp == NULL ) {
		std::cout << "Directory does not exist" << std::endl; 
		return; 
	}

	while( entry = readdir(dp) ) {

		files.push_back(entry->d_name); 

	}

	closedir(dp); 

}



	
