#ifndef CAND_VIS
#define CAND_VIS

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

		int midx = 0.5*(it->xmin+it->xmax);
		int midy = 0.5*(it->ymin+it->ymax);
	
	//	std::cout << "Candidate " << it->id << ":" << std::endl;
	
	/*	std::cout << "Inlier fraction: " << it->max_inlier_fraction << std::endl;
		std::cout << "Height: " << it->real_height << std::endl;
		std::cout << "Width: " << it->real_width << std::endl;
		std::cout << "Density: " << ((float)it->size() / (p_height * p_width)) << std::endl; 
*/

	//	std::cout << "Centre coords x: " << midx; 
	//	std::cout << " y: " << midy << std::endl; 

/*		std::cout << "Centre x: " << it->centre.x; 
		std::cout << " y: " << it->centre.y; 
		std::cout << " z: " << it->centre.z << std::endl; 

		std::cout << "Height y: " << it->real_height; 
		std::cout << " x: " << it->real_width << std::endl; */

		/* std::ostringstream cand_name; 
		cand_name << "Candidate " << i; 
		displayImg(it->im, cand_name.str()); */

		i++; 

	}

	displayImg(candidate_img); 

}

#endif

