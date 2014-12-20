// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include "segmenter/segmenter.h"

void displayImg(cv::Mat& img)
{
	// scale values to 8-bit
	cv::Mat img_disp = img.clone();
	double minval, maxval;
	cv::minMaxIdx(img_disp, &minval, &maxval);

	std::cout << "Minval: " << minval << " Maxval" << maxval << std::endl;

	img_disp.convertTo(img_disp, CV_8UC1, 255.0/maxval);

	// show image
	cv::namedWindow("PUKA", 1);
	cv::imshow("PUKA", img_disp);
	cv::waitKey();
}

int main(int argc, char** argv)
{
	if (argc != 2)
		std::cout << "No file specified" << std::endl;

	// load file
	cv::FileStorage file(argv[1], cv::FileStorage::READ);

	// load image to matrix
	cv::Mat img;
	file["puka"] >> img;	

	// display on screen
	displayImg(img);

	// segment
	segmentImg(img);
}
