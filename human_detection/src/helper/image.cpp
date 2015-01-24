#include <sstream>

#include <iostream>
#include <dirent.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"

// Displays an image using OpenCV and waits for user
void displayImg(cv::Mat& img, std::string window_name = "")
{

	static int win_num = 0; 
	std::ostringstream win_name; 

	// scale values to 8-bit
	cv::Mat img_disp = img.clone();
	double minval, maxval;
	cv::minMaxIdx(img_disp, &minval, &maxval);

	img_disp.convertTo(img_disp, CV_8UC1, 255.0/maxval);

	if( window_name.empty() ) {

		win_name << "PUKA" << win_num;

		// show image
		cv::namedWindow(win_name.str(), CV_WINDOW_NORMAL);
		cv::imshow(win_name.str(), img_disp);

		win_num++; 

	} else {

		cv::namedWindow(window_name.c_str(), CV_WINDOW_NORMAL);
		cv::imshow(window_name.c_str(), img_disp);

	}

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
	struct stat entrystat;

	dp = opendir(path.c_str()); 

	if( dp == NULL ) {
		std::cout << "Directory does not exist" << std::endl; 
		return; 
	}

	while( entry = readdir(dp) ) {

		if( stat((path + "/" + entry->d_name).c_str(), &entrystat ) != 0 ) 
			std::cout << "Error getting file details" << std::endl; 

		if( S_ISREG( entrystat.st_mode ) ) {
			files.push_back(path + "/" + entry->d_name); 	
		} 

		if( entry->d_type & DT_DIR ) {

			if( strcmp(entry->d_name, "..") != 0 && strcmp(entry->d_name, ".") != 0 ) {

				std::cout << "Opening directory " << entry->d_name << std::endl;

				// Is a directory
				directory_list(files, path + "/" + entry->d_name);

			}

		}

	}

	closedir(dp); 

}



	
