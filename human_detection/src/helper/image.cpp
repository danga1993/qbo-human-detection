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
void displayImg(cv::Mat& img, std::string window_name)
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


// Display a Felzenimage (RGB)
void display_felzen( image<rgb> * img) {

  cv::Mat img_disp(img->height(), img->width(), CV_32FC3); 

	// Loop through and convert to matrix
  int img_height = img->height(); 
  int img_width = img->width(); 

  for( int y = 0; y < img_height; y++ ) {
    for( int x = 0; x < img_width; x++ ) {

      img_disp.at<cv::Vec3f>(y, x)[0] = (float)imRef(img, x, y).b; 
			img_disp.at<cv::Vec3f>(y, x)[1] = (float)imRef(img, x, y).g; 
			img_disp.at<cv::Vec3f>(y, x)[2] = (float)imRef(img, x, y).r; 		

    }
  }

  // Display
  displayImg(img_disp); 

}

// Display a Felzenimage (float)
void display_felzen( image<float> * img) {

  cv::Mat img_disp(img->height(), img->width(), CV_32FC1); 

	// Loop through and convert to matrix
  int img_height = img->height(); 
  int img_width = img->width(); 

  for( int y = 0; y < img_height; y++ ) {
    for( int x = 0; x < img_width; x++ ) {

      img_disp.at<float>(y, x) = (float)imRef(img, x, y); 

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


void display_candidates(int width, int height, std::vector<candidate>& candidates) { 

	// Generate image of candidates
	cv::Mat candidate_img = cv::Mat::zeros(height+10, width+50, CV_32FC3); 

	int i = 0; 
	// Fill in image from candidates
	for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {

		if( it->erased ) 
			continue; 

		// Generate a colour for the candidate
		rgb cand_colour = random_rgb(); 

		// Offset of candidate in image
		//cv::Point origin_segmented = cv::Point(it->xmin, it->ymin);

		for (std::vector<cv::Point3f>::iterator it1 = it->pts.begin(); it1 != it->pts.end(); it1++){
			candidate_img.at<cv::Vec3f>(it1->y, it1->x) = cv::Vec3f(cand_colour.b, cand_colour.g, cand_colour.r); 

		}

		cv::Scalar cand_colour_scalar = cv::Scalar(cand_colour.b, cand_colour.g, cand_colour.r);

		// Draw a rectangle around the candidate
		//cv::rectangle(candidate_img, cv::Point(it->xmin, it->ymin), cv::Point(it->xmax, it->ymax), cand_colour_scalar); 

		// Add text label
		std::ostringstream cand_label; 
		cand_label << it->id; 
		cv::putText(candidate_img, cand_label.str(), cv::Point(width+1 + 12*(i%4), 7*(i/4+1)), cv::FONT_HERSHEY_PLAIN, 0.5, cand_colour_scalar);  

		int p_height = it->ymax - it->ymin;
		int p_width = it->xmax - it->xmin;
	
		std::cout << "Candidate " << it->id << ":" << std::endl;
	
	/*	std::cout << "Inlier fraction: " << it->max_inlier_fraction << std::endl;
		std::cout << "Height: " << it->real_height << std::endl;
		std::cout << "Width: " << it->real_width << std::endl;
		std::cout << "Density: " << ((float)it->size() / (p_height * p_width)) << std::endl; */

		std::cout << "Centre x: " << it->centre.x; 
		std::cout << " y: " << it->centre.y; 
		std::cout << " z: " << it->centre.z << std::endl;

		/* std::ostringstream cand_name; 
		cand_name << "Candidate " << i; 
		displayImg(it->im, cand_name.str()); */

		i++; 

	}

	displayImg(candidate_img); 

}





	
