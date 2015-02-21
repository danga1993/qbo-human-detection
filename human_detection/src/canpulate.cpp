#include <iostream>
#include <stdio.h>
#include <sys/stat.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "helper/image.h"

#include "config.h"

#include "canpulate.h"

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
	
	if( argc < 2 ) {
		std::cout << "Enter command"; exit(1); 
	}

	std::string command(argv[1]); 

	std::cout << command << std::endl;

	// Mirror candidates
	if( command == "mirror" ) {

		std::cout << "Mirroring" << std::endl;

		std::vector<std::string> files; 

		// Get list of files
		directory_list(files, "train_data/positive"); 
		directory_list(files, "train_data/negative", false); 

		// Loop through and mirror
		for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

			std::string dest(*it + "m");

			// Create mirrored directory
			make_directory(dest); 

			// Mirror and move candidate
			mirror_candidate(*it + "/depth.png", dest + "/depth.png"); 
	
		}

	}

	
	// Clear folders
	if( command == "clean" ) {

		clean_candidate_dirs(); 

	}

	if( command == "generate" ) {

		generate_set(); 

	}
		

	return 0;
}

// Moves a candidate and associated files
void mirror_candidate(std::string source, std::string dest) {

	cv::Mat img; 

	// Copy scale factor
	fs::copy_file( fs::path(source + ".mat"), fs::path(dest + ".mat")  ); 

	img = cv::imread(source, CV_LOAD_IMAGE_ANYDEPTH); 

	// Mirror candidate
	cv::flip(img, img, 1); 

	// Compression parameters
	std::vector<int> png_params; 
	png_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	png_params.push_back(9); 		// Higher takes longer but smaller

	imwrite(dest, img, png_params);

}

// Cleans candidate directories for training and testing
void clean_candidate_dirs() {

		// Training data
		clean_directory("train_data/positive"); 
		clean_directory("train_data/negative");

		// Test data
		clean_directory("test_data/positive");
		clean_directory("test_data/negative");

		// Misses
		clean_directory("test_misses/positive");
		clean_directory("test_misses/negative");

}



// Randomly separates candidates into a test and training set
void generate_set() {

	std::vector<std::string> files; 

	// Clean the target directories
	clean_candidate_dirs(); 

	// Retrieve candidates
	directory_list(files, "candidates/positive"); 
	directory_list(files, "candidates/negative", false); 

	// Seed random generator
	srand(time(NULL));

	// Loop through and form sets
	for (std::vector<std::string>::iterator it = files.begin(); it != files.end(); it++) {

		// Strip initial part of path
		std::string cand_path = (*it).substr((*it).find_first_of("/")+1, std::string::npos); 

		// Randomly decide split (1:5)
		int test_set = ((std::rand() % 5) == 0); 

		// Place data into correct set
		std::string dest = ((test_set) ? "test_data/" : "train_data/") + cand_path; 

		// Copy
		copy_dir(*it, dest); 

	}

}
		
	  
	




