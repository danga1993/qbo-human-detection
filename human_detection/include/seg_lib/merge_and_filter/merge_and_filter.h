#include "config.h"
#include <opencv2/core/core.hpp>
#include "candidate-implement.h"
#include <vector>
#include <algorithm>    // std::min
#include <stdio.h>
#include <iostream>
#include <map>

#ifndef MERGENFILTER_H
#define MERGENFILTER_H

// Merging methods
void original_merge(std::vector<candidate>& candidates);
void gaussian_merge(std::vector<candidate>& candidates);

void merge_edges(candidate& dest, candidate& src, std::vector<candidate>& candidates);

// Performs merging and filtering of candidates
void merge_and_filter(image<float> *im, universe * u, int width, int height, cv::Mat &depthim, std::vector<candidate>& candidates){

	std::map<int,candidate> components;

	// Clear candidate vector
	candidates.clear(); 

	//iterator for searching use
	std::map<int,candidate>::iterator it;

	//loop through each pixel in the image and add defined values
	for(int y=0; y < height; y++){
		for(int x = 0; x < width; x++){
			float z = imRef(im,x,y);
			if( abs(z) > VALID_VALUE){
							//get component id for pixel at (x,y)
				int id = u->find(y*width+x); //refer to the colouring algorithm in segment-image.h

				it = components.find(id);
				if(it == components.end()){
					//if id doesnt exist in map yet...
					//add new component with first pel at x,y, to the map
					candidate c = candidate(x,y,z,id);
					components.insert(std::map<int,candidate>::value_type(id, c));
				}
				else {
					//just add pel to candidate
					it->second.add(x,y,z);
				}
			}
		}
	}

	//now check that for the same id's u->size(x) == candidate.size.
	/*	
	for(int y=0; y < height; y+=10){
		for(int x = 0; x < width; x+=10){
			int id = u->find(y*width+x);
			std::map<int,candidate>::iterator it;
			it = components.find(id);
			std::cout << "Component: " << id << " has size: " << u->size(id) << std::endl;
			std::cout << "Candidate: " << id << " has size: " << it->second.size() << std::endl;
		}
	}
	//*/

	//std::cout << "Components " << components.size() << std::endl; 

	//std::map<int,candidate>::iterator local_vertex;
	//std::map<int,candidate>::iterator edge_vertex;

	candidate * local_vertex; 
	candidate * edge_vertex; 

	// Add up number of edges for each component
	for(int y=0; y < height; y++) {
		for(int x = 0; x < width; x++) {

			// Vertex (x,y)
			local_vertex = &(components.find(u->find(y*width+x))->second);

			if (x < width-1) {
				edge_vertex = &(components.find(u->find(y * width + (x+1)))->second);
				local_vertex->add_edge(edge_vertex->id); 
				edge_vertex->add_edge(local_vertex->id); 
		  }

	    if (y < height-1) {
				edge_vertex = &(components.find(u->find((y+1) * width + x))->second);
				local_vertex->add_edge(edge_vertex->id); 
				edge_vertex->add_edge(local_vertex->id); 
	    }

	    if ((x < width-1) && (y < height-1)) {
				edge_vertex = &(components.find(u->find((y+1) * width + (x+1)))->second);
				local_vertex->add_edge(edge_vertex->id); 
				edge_vertex->add_edge(local_vertex->id); 
	    }

	    if ((x < width-1) && (y > 0)) {
				edge_vertex = &(components.find(u->find((y-1) * width + (x+1)))->second);
				local_vertex->add_edge(edge_vertex->id); 
				edge_vertex->add_edge(local_vertex->id); 
	    }

		}
	}


	//for each candidate, calculate the centres of mass, and reject those candidates which are not sufficiently planar
	for(it = components.begin(); it != components.end(); it++){
		//calculate params for heuristic comparisons TODO we can remove the variables from the class have them just return here
		it->second.calc_centre(); 					// but its nice to have them in the class for now for reference
		it->second.RANSAC_inliers();
		it->second.calc_real_dims();
		float fraction = it->second.max_inlier_fraction;
		float height = it->second.real_height;
		float width = it->second.real_width;
		
		/*std::cout << "Size: " << it->second.size() << ", Depth: " << it->second.centre.z << ", w: " << width << ", h: " << height << ", fraction: " << fraction << std::endl;
		std::cout << "Height < MAX_HEIGHT: " << (height < CANDIDATE_MAX_HEIGHT) << std::endl;
		std::cout << "Width < MAX_WIDTH: " << (width < CANDIDATE_MAX_WIDTH) << std::endl;
		std::cout << "Fraction < MIN_INLIER_FRACTION: " << (fraction > MIN_INLIER_FRACTION) << std::endl;
		//*/
		
		//if components satisfy these, add them to candidates list
		if( ((fraction > MIN_INLIER_FRACTION) && (height < CANDIDATE_MAX_HEIGHT) && (width < CANDIDATE_MAX_WIDTH)) ){
			 //std::cout << "Keep Candidate: " << it->second.id << " with fraction " << fraction << std::endl;
			candidates.push_back(it->second);
		} else {
			// std::cout << "Remove Candidate: " << it->second.id << " with fraction " << fraction << " height: " << height << " width: " << width << std::endl;
		}
		
	}

	//free up some memory
	components.clear();

	//sort vector into order, largest first. (given we have overloaded the '<' operator to work with size)
	std::sort(candidates.rbegin(), candidates.rend());

	
	// Re ID the candidates
	{ 
		int i = 0; 

		std::map<int,int> cand_map; 

		// Create mapping from old -> new id
		for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++, i++) {
			cand_map.insert(std::pair<int,int>(itc->id, i));  	
			//std::cout << "Mapping " << itc->id << " to " << i << std::endl; 
			itc->id = i;
		}

		//std::cout << "Re-id candidates" << std::endl;

		// Update edges
		for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++) {

			// Build new edge map
			std::map<int,int> edge_map; 	

			for(std::map<int,int>::iterator it1 = itc->edges.begin(); it1 != itc->edges.end(); it1++) {
				if( cand_map.count(it1->first) ) {
					edge_map.insert(std::pair<int,int>(cand_map.at(it1->first), it1->second)); 
				}
			}

			// Swap in new map
			itc->edges = edge_map; 

		}	
	} 

	//std::cout << "Displaying candidates" << std::endl;
	
	// Display the candidates before merge
	display_candidates(width, height, candidates); 
	cv::waitKey();

	switch( MERGE_METHOD ) {
		case MERGE_ORDINARY: 
			original_merge(candidates); 
			break; 
		case MERGE_GAUSSIAN: 
			gaussian_merge(candidates); 
			break; 
	}
	
	
	
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){
		//std::cout << "Candidate Size: " << itc->size() << std::endl;
		if( (!itc-> erased) ){

			int p_height = itc->ymax - itc->ymin;
			int p_width = itc->xmax - itc->xmin;
			int size = itc->size();

			itc->crimp_boundingbox(width, height);
		
			//std::cout << "Evaluating Candidate of Height: " << itc->real_height << ", Width: " << itc->real_width << std::endl;
			//std::cout << "Height < MIN_HEIGHT: " << (itc->real_height < CANDIDATE_MIN_HEIGHT) << std::endl;
			//std::cout << "Width < MIN_WIDTH: " << (itc->real_width < CANDIDATE_MIN_WIDTH) << std::endl;
			//now reject any candidates that are still minimum post merge
			//if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) || ((float)size/(p_height*p_width) < CANDIDATE_MIN_DENSITY) ){
			if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) || (itc->box.width <= 0) || (itc->box.height <= 0) ) {
			//if( (itc->box.width <= 0) || (itc->box.height <= 0) ) {
				//std::cout << "Erased " << itc->id << std::endl;
				itc->erased = true;
			} 
		}
	}
	//*/

	//finally convert images into candidate images and calculate their bounding boxes
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){
		if( (!itc-> erased) ){
			//initialise the bounding box
			itc->set_boundingBox();
			itc->crimp_boundingbox(width, height);
			itc->create_candidate_image(depthim);
			itc->fullscale_boundingBox(); 

			//std::cout << "Displaying candidate" << std::endl;

			//displayImg(itc->im); 
			//cv::waitKey();
		}
	}

}


// Merges using gaussians
void gaussian_merge(std::vector<candidate>& candidates) { 

	float sigma = 0.4;   // 0.4
	float threshold_x = 1.6; 
	float threshold_z = 0.5; 
	float threshold_y = 1.0; 
	float weight_factor = 0.15; 

	// NOTE: THIS LOOP MIGHT NEED TO BE REPEATED
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){

		//std::cout << "Candidate Size: " << itc->size() << std::endl;
		int p_height = itc->ymax - itc->ymin;
		int p_width = itc->xmax - itc->xmin;
		int size = itc->size();

		if(p_height*p_width == 0){
			//std::cout << "Zero area found" << std::endl;
		} 

		/* std::cout << "Id: " << itc->id << std::endl;
		std::cout << "Height: " << itc->real_height;
		std::cout << " Width: " << itc->real_width;
		std::cout << " Density: " << ((float)size / (p_height * p_width)) << std::endl; */

		//if the candidate is smaller than the minimum width or height
		//if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) || ((float)size/(p_height*p_width) < CANDIDATE_MIN_DENSITY) ){
			//search for larger candidates (i.e. from end() to where we are now; itc)
			cv::Point2f centrexz(itc->centre.x, itc->centre.z);
			///*		

	
			//for(std::vector<candidate>::iterator itc1 = candidates.begin(); *itc1 > *itc ; itc1++){
			int n = 15; 
			for(std::vector<candidate>::iterator itc1 = candidates.begin(); (n > 0 && *itc1 > *itc); itc1++, n--){
				if (!itc1->erased){

					float weight = 0; 
			
					// Estimate merge factor for candidate
					for (std::vector<cv::Point3f>::iterator itpt = itc->pts.begin() ; itpt != itc->pts.end(); ++itpt){
						float x,y;
						itc->calc_real_coords(itc->centre.z, itpt->x*ALPHA, itpt->y*ALPHA, x, y); 

						weight += exp( abs(x - itc1->centre.x) / sigma ); 
			
					}

					weight /= itc->size(); 

					float y_distance = abs(itc->centre.y-itc1->centre.y);
					float z_distance = abs(itc->centre.z-itc1->centre.z);

					if( itc->id < 25 ) {
				  	// Merge if below threshold
						//std::cout << "Merge " << itc->id << " ( " << itc->centre.x << "," << itc->centre.y << ", " << itc->centre.z << ") -> " << itc1->id << " ( " << itc1->centre.x << "," << itc1->centre.y << ", " << itc1->centre.z << ")" << std::endl;
						//std::cout << "Weight: " << weight << std::endl;
					}
			
						if( itc1->edges.count(itc->id) ) {

							float edge_factor = exp( -((float)itc1->edges.at(itc->id) / size) * weight_factor / sigma ); 
							// Add to weight
							weight *= edge_factor; 

							if( itc->id < 25 ) {
								//std::cout << " Edges: " << itc1->edges.at(itc->id);
								//std::cout << " Edge fraction: " << (((float)itc1->edges.at(itc->id) / size));
								//std::cout << " Edge factor: " << edge_factor;
								//std::cout << " New Weight: " << weight << std::endl;
							}

						}

					if( weight < threshold_x && y_distance < threshold_y && z_distance < threshold_z ) {
		
						// std::cout << buf.str(); 				

						 itc1->merge(*itc, candidates); //merge itc into itc1
						 itc->erased = true; //and erase itc

						break;

					}

				}

			}

		//} 

	}
						
}


// Merges as done in research paper
void original_merge(std::vector<candidate>& candidates) { 

	// NOTE: THIS LOOP MIGHT NEED TO BE REPEATED
	for(std::vector<candidate>::iterator itc = candidates.begin(); itc != candidates.end(); itc++){

		//std::cout << "Candidate Size: " << itc->size() << std::endl;
		int p_height = itc->ymax - itc->ymin;
		int p_width = itc->xmax - itc->xmin;
		int size = itc->size();

		if(p_height*p_width == 0){
			//std::cout << "Zero area found" << std::endl;
		} 

		/* std::cout << "Id: " << itc->id << std::endl;
		std::cout << "Height: " << itc->real_height;
		std::cout << " Width: " << itc->real_width;
		std::cout << " Density: " << ((float)size / (p_height * p_width)) << std::endl; */

		//if the candidate is smaller than the minimum width or height
		if( (itc->real_width < CANDIDATE_MIN_WIDTH) || (itc->real_height < CANDIDATE_MIN_HEIGHT) || ((float)size/(p_height*p_width) < CANDIDATE_MIN_DENSITY) ){
			//search for larger candidates (i.e. from end() to where we are now; itc)
			cv::Point2f centrexz(itc->centre.x, itc->centre.z);
			///*			
			for(std::vector<candidate>::iterator itc1 = candidates.begin(); *itc1 > *itc ; itc1++){
				if (!itc1->erased){
					//std::cout << "Test Candidate Size: " << itc1->size() << std::endl;
					//merge if the following conditions are met
					cv::Point2f testxz(itc1->centre.x, itc1->centre.z);
					//std::cout<<"Point: x " << itc1->centre.x << " y " << testxz.y << std::endl;
					float xz_distance = cv::norm(cv::Mat(centrexz),cv::Mat(testxz));
					float y_distance = abs(itc->centre.y-itc1->centre.y);
					//std::cout<<"Proximity: delxz =" << xz_distance << " dely =" << y_distance << std::endl;

					//std::cout << "Merge " << itc->id << " -> " << itc1->id; 
					//std::cout << " XZ: " << xz_distance << " Y: " << y_distance << std::endl;
		
					if( (xz_distance < DELTAXZ) && (y_distance < DELTAY) ){
						 itc1->merge(*itc, candidates); //merge itc into itc1
						 itc->erased = true; //and erase itc
						 //std::cout << "Merged" << std::endl;
						 break;
						 //std::cout<<"Merged Candidate " << itc->id << " into " << itc1->id <<std::endl;	
					}
				
				}
			}//*/
		}
		else{
			//std::cout << "Candidate Already Valid" << std::endl;
		}
	}

}




#endif
