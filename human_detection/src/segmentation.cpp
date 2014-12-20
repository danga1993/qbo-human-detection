// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
	if (argc != 2)
		std::cout << "No file specified" << std::endl;

	// load file
	cv::FileStorage file(argv[1], cv::FileStorage::READ);

	// load image to matrix
	cv::Mat img;
	file["puka"] >> img;	

	// scale values to 8-bit
	cv::Mat img_out = img.clone();
	double minval, maxval;
	cv::minMaxIdx(img_out, &minval, &maxval);

	std::cout << "Minval: " << minval << " Maxval" << maxval << std::endl;

	img_out.convertTo(img_out, CV_8UC1, 255.0/maxval);
	// cv::cvtColor(img_out, img_out, cv::GRAY2BGR);
	
	// show image
	cv::namedWindow("PUKA", 1);
	cv::imshow("PUKA", img_out);
	cv::waitKey();
}
