// segmentation

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "seg_lib/segment_depth/segment.h"

#include "helper/image.h"
#include "tagger/tagger.h"
#include "config.h"

#include "segmenter/segmenter.h"

int main(int argc, char** argv)
{

	cv::FileStorage file;
	std::vector<std::string> files; 

	// Segmentation candidates
	std::vector<candidate> candidates; 

	// Bouding boxes for pre-tagging candidates
	std::vector<cv::Rect> bounding_boxes; 

	// Get list of files
	directory_list(files, "frames"); 

	// Loop through 
	for (std::vector<std::string>::iterator file_it = files.begin(); file_it != files.end(); file_it++) {

		// load file
		file.open(*file_it, cv::FileStorage::READ);

		// load image to matrix
		cv::Mat img;
		file["puka"] >> img;	

		// segment
		Segmenter::segment(img, candidates);

		// Tag the candidates
		if( TAG_AUTO ) {

			// Retrieve bounding boxes for image
			//if( file["boundingbox"].type() == cv::FileNode::USER ) {

			file["boundingbox"] >> bounding_boxes; 

			if( bounding_boxes.size() > 0 ) {

				std::cout << "Box " << bounding_boxes.at(0) << std::endl;

				Tagger::tag(candidates, bounding_boxes); 


			} else {
				std::cout << "No bounding box defined for candidate" << std::endl; 
			}

		} else {

			Tagger::tag(candidates); 

		}
			
		file.release(); 
	
		// Save the candidates
		std::stringstream fname; 
		static int i = 0; 

		for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {

			if( !it->erased ) {

				// Generate filename
				fname.str(""); 
				fname << "candidates/" << ((it->human) ? "positive/" : "negative/") << i << ".mat"; 

				// Open file
			 	file.open(fname.str(), cv::FileStorage::WRITE);

				file << "image" << it->im; 
				file << "human" << it->human;

				file.release();

				i++; 
		
			}

		}

	}
		

}
