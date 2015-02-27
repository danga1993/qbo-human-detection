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

	cv::Mat img; 

	// Segmentation candidates
	std::vector<candidate> candidates; 

	// Bounding boxes for pre-tagging candidates
	std::vector<cv::Rect> bounding_positive; 
	std::vector<cv::Rect> bounding_negative; 

	// Get list of files
	directory_list(files, "frames"); 

	// Loop through 
	for (std::vector<std::string>::iterator file_it = files.begin(); file_it != files.end(); file_it++) {

		// Load depth image
		image_read(*file_it + "/depth.png", img); 

		// Load bounding box data
		file.open(*file_it + "/data.mat", cv::FileStorage::READ);

		// If manually segmenting or automatic
		if( SEGMENT_AUTO ) {

			// Segment
			Segmenter_Auto::segment(img, candidates);

			// Tag the candidates
			if( TAG_AUTO ) {

				// Retrieve bounding boxes for image
				//if( file["boundingbox"].type() == cv::FileNode::USER ) {

				file["bounding_positive"] >> bounding_positive; 

				if( bounding_positive.size() > 0 ) {

					std::cout << "Box " << bounding_positive.at(0) << std::endl;

					Tagger::tag(candidates, bounding_positive); 

					// Remove negative candidates from frame with positives
					for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {
						if( !it->human && !it->erased ) {
							it->erased = true; 
						}
					}
			
				} else {
					//std::cout << "No bounding box defined for candidate" << std::endl; 
				}

			} else {

				Tagger::tag(candidates); 

			}

		// Manual segmentation (tagging implicit)
		} else {

			file["bounding_positive"] >> bounding_positive; 
			file["bounding_negative"] >> bounding_negative;

			// Segment and tag
			Segmenter_Manual::segment(img, candidates, bounding_positive, bounding_negative); 

		}			
			
		file.release(); 
	
		// Save the candidates
		std::stringstream fname; 
		static int i = 0; 

		for (std::vector<candidate>::iterator it = candidates.begin(); it != candidates.end(); it++) {

			if( !it->erased ) {

        std::cout << "Saving candidate" << std::endl;
	
				// Generate filename
				fname.str(""); 
				fname << "candidates/" << ((it->human) ? "positive/" : "negative/") << i; 

				make_directory(fname.str()); 
				
			/*displayImg(it->im); 
				cv::waitKey(); */

				image_write(fname.str() + "/depth.png", it->im);  

				i++; 
		
			}

		}

	}
		

}
