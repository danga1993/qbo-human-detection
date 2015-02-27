#include "candidate.h"
#include "helper/image.h"
#include "candidate-visualize.h"

void showImg(cv::Mat& img)
{

	static int win_num = 0; 
	std::ostringstream win_name; 

	// scale values to 8-bit
	cv::Mat img_disp = img.clone();
	double minval, maxval;
	cv::minMaxIdx(img_disp, &minval, &maxval);

	img_disp.convertTo(img_disp, CV_8UC1, 255.0/maxval);

	win_name << "PUKAS" << win_num;

	// show image
	cv::namedWindow(win_name.str(), 1);
	cv::imshow(win_name.str(), img_disp);

	win_num++; 
}


inline bool operator<(const candidate& lhs, const candidate& rhs){
	int lhsSize = lhs.size();
	int rhsSize = rhs.size();
	return lhsSize<rhsSize;
}

inline bool operator>(const candidate& lhs, const candidate& rhs){
	return operator<(rhs,lhs);
}

inline bool operator<=(const candidate& lhs, const candidate& rhs){
	return !operator>(lhs,rhs);
}

inline bool operator>=(const candidate& lhs, const candidate& rhs){
	return !operator<(lhs,rhs);
}

void candidate::calc_real_coords(float depth, int x, int y, float& real_x, float& real_y) {
	
	//float pel_ratio = (float)pel/(length - 1.0);
	//float angular_component = tan(fov/2);
	//return depth*(pel_ratio - 0.5)*angular_component*2; //I think there should be an extra factor of 2 in this term

	real_x = ((float)x/(FRAME_WIDTH-1) - 0.5) * tan(FOV_H/2) * 2 * depth; 
	real_y = ((float)y/(FRAME_HEIGHT-1) - 0.5) * tan(FOV_V/2) * 2 * depth; 

	//float pel_ratio = float(pel_from_lower_left)/length;
	//float metric_scene_length = depth*2*tan(fov/2);
	//return pel_ratio*metric_scene_length;	
}

candidate::candidate(int x,int y, float z, int i){
	xmin = x;
	ymin = y;
	xmax = x;
	ymax = y;
	depth_accumulator = z;
	boundingBox = cv::Rect(xmin,ymin,xmax-xmin,ymax-ymin);
	erased = false; //TODO
	merged = false;
	human = false;
	id = i;
	max_inlier_fraction = 0;
	//std::cout << "New Candidate: (" << x << "," << y << "," << z << ") with id: " << id << std::endl;
	pts.push_back(cv::Point3f(x,y,z));
	classification = -1;
}

candidate::candidate(cv::Mat& input_im, bool input_human) {

	im = input_im.clone(); 
	human = input_human; 
	erased = false;
}

candidate::candidate(){
}

candidate::~candidate(){

}

void candidate::add(int x, int y, float z){
	//TODO given that we are now keeping a record of the pts for each candidate, 
	//find bounding box variables	
	xmin = std::min(x,xmin);
	xmax = std::max(x,xmax);
	ymin = std::min(y,ymin);
	ymax = std::max(y,ymax);
	depth_accumulator += z;

	/*
	//calculate real position
	float rx = z*( (x/(PEL_WIDTH-1) -0.5) )*tan(F_H/2); //I think there should be an extra factor of 2 in this term
	float ry = z*( (y/(PEL_HEIGHT-1) -0.5) )*tan(F_V/2); //I think there should be an extra factor of 2 in this term
	float rz = z;
*/

	//std::cout << "Add: (" << x << "," << y << "," << z << ") to candidate with id: " << id << std::endl;
	pts.push_back(cv::Point3f(x,y,z));
}

void candidate::set_boundingBox(){
	boundingBox = cv::Rect(xmin*ALPHA,ymin*ALPHA,(xmax-xmin+1)*ALPHA,(ymax-ymin+1)*ALPHA);
}

int candidate::size() const{
	return pts.size();
}

void candidate::calc_real_dims(){

	float xleft, ytop, xright, ybottom; 

	calc_real_coords(centre.z, xmin*ALPHA, ymin*ALPHA, xleft, ytop);
	calc_real_coords(centre.z, xmax*ALPHA, ymax*ALPHA, xright, ybottom);

	real_height = ybottom - ytop;
	real_width = xright - xleft;

	/*std::cout << "Xmin: " << xmin << ", xmax: " << xmax << ", ymin: " << ymin << ", ymax: " << ymax <<std::endl;
	std::cout << "Depth: " << centre.z << " Xleft: " << xleft << ", xright: " << xright << ", ytop: " << ytop << ", ybottom: " << ybottom <<std::endl;*/
	//std::cout << "Candidate: " << id << ", real_height: " << real_height << ", real_width: " << real_width << std::endl;

}
	
void candidate::create_candidate_image(cv::Mat &depthim){
	//a value of -1 is invalid.

	int width = xmax-xmin+1;
	int height = ymax-ymin +1;

	cv::Mat oscm = cv::Mat::zeros(cv::Size(width,height),CV_8UC1); //original scale candidate mask
	//std::cout << "Size of oscm: width = " << oscm.size().width << ", height = " << oscm.size().height << std::endl; 
	//std::cout << "xmin = " << xmin <<", xmax = " << xmax << ", ymin = " << ymin <<", ymax = " << ymax << std::endl;

	//given that we only have a list of points, we first need to build the image representing these points
	for (std::vector<cv::Point3f>::iterator it = pts.begin(); it != pts.end(); it++){
		//std::cout << "x: " << it->x - xmin << ", y: " << it->y - ymin << std::endl;		
		oscm.at<uchar>(cv::Point(it->x-xmin,it->y-ymin)) = 1;
		//std::cout<< "Mask val: " << oscm.at<uchar>(cv::Point(it->x-xmin,it->y-ymin)) << std::endl;
	}

	// Original scale candidate image
	cv::Mat nscm = cv::Mat(cv::Size(width*ALPHA, height*ALPHA), CV_32FC1, 0.0); //New Scale Candidate Mask

	//std::cout << "Scaling: " << scaling << ", new: width = " << nscm.size().width << ", height = " << nscm.size().height << std::endl; 

	// Fill in this new scaled image
	for(int x = 0; x < nscm.size().width; x++){
		for(int y = 0; y < nscm.size().height; y++){
			int oldx = x/ALPHA;
			int oldy = y/ALPHA;

			if( oscm.at<uchar>(cv::Point(oldx,oldy)) == 1 ) {

				// Position of pixel in original image
				cv::Point depth_pel = cv::Point(xmin, ymin)*ALPHA + cv::Point(x, y); 
	
				nscm.at<float>(cv::Point(x,y)) = depthim.at<float>(depth_pel); 

			}

		}
	}

	// Fit image into candidate window
	position_candidate_image(im, nscm); 

}

// Generates candidate image from region (bypasses segmentation)
void candidate::create_candidate_image(cv::Mat &depthim, cv::Rect& region) { 

		// Clip region to image - NOTE: THIS SHOULD BE DONE IN LABELLING
		region.width = std::min(region.width, depthim.size().width - region.x); 
		region.height = std::min(region.height, depthim.size().height - region.y); 

		// Copy region into matrix
		cv::Mat nscm = cv::Mat(cv::Size(region.width, region.height), CV_32FC1, 0.0); 

		depthim(cv::Rect(region.x,region.y,region.width,region.height)).copyTo(nscm); 

		// Fit image into candidate window
		position_candidate_image(im, nscm);

		//std::cout << "Filled" << std::endl;

}
				
		
// Positions a candidate image inside the candidate window
void candidate::position_candidate_image(cv::Mat& cand_window, cv::Mat& cand_image) {

	cand_window = cv::Mat(cv::Size(CANDIDATE_WIDTH, CANDIDATE_HEIGHT), CV_32FC1, 0.0/0.0);
	cv::Size dims = cand_image.size(); 

	//displayImg(cand_image); 
	//cv::waitKey(); 

	// Scale and fit the candidate image into the window
	if( CANDIDATE_IMAGE_SCALE ) {

		// Calculate scaling factors
		float scale_x = (float) CANDIDATE_WIDTH / dims.width; 
		float scale_y = (float) CANDIDATE_HEIGHT / dims.height; 

		// Loop through and fill in candidate
		for( int x = 0; x < CANDIDATE_WIDTH; x++ ) {
			for( int y = 0; y < CANDIDATE_HEIGHT; y++ ) {

				cand_window.at<float>(cv::Point(x, y)) = cand_image.at<float>(cv::Point(x/scale_x, y/scale_y)); 

			}
		}

	} else {

		float scaling = std::min((float)CANDIDATE_WIDTH/dims.width, (float)CANDIDATE_HEIGHT/dims.height);
		
		cv::Size scaled_cand = cv::Size(dims.width*scaling, dims.height*scaling); 

		cv::Point image_centre = cv::Point((CANDIDATE_WIDTH)/2, (CANDIDATE_HEIGHT)/2);
		cv::Point start = image_centre - cv::Point(scaled_cand.width/2, scaled_cand.height/2);

		for(int x = 0; x < scaled_cand.width; x++){
			for(int y = 0; y < scaled_cand.height; y++){
				cand_window.at<float>(start+cv::Point(x,y)) = cand_image.at<float>(cv::Point(x/scaling,y/scaling));
			}
		}

	}

}

bool candidate::merge(candidate c){
	//note that c should be of a larger size than this
	if( c.size() > size() ){
		std::cout << "MERGE FAILED: CAN ONLY MERGE WITH A SMALLER CANDIDATE" << std::endl;
		return false;
	}
	//find out new bounding box params	
	xmin = std::min(xmin, c.xmin);
	xmax = std::max(xmax, c.xmax);
	ymin = std::min(ymin, c.ymin);
	ymax = std::max(ymax, c.ymax);

	//use these to recalculate the real dimensions
	calc_real_dims();
	calc_centre();

	//add the points
	//pts.insert(pts.end(), c.pts.begin(), c.pts.end());
	//std::cout << "Add Points";
	int count = 0;
	for (std::vector<cv::Point3f>::iterator it = c.pts.begin() ; it != c.pts.end(); ++it){
		pts.push_back(*it);
		//std::cout << *it << ", ";
		count++;
		depth_accumulator+=it->z;
	}
	//std::cout << "Pixels Added: " << count << std::endl;
	merged = true;
	return true;

}

/*void candidate::add_to_image(image<rgb> * im, rgb colour){
	
	for(std::vector<cv::Point3f>::iterator it = pts.begin(); it != pts.end(); it++){
      imRef(im, it->x, it->y) = colour;
	}

}*/
void candidate::RANSAC_inliers(){

	// 3 points needed to define a plane
	if( size() > 3 ) {

		//find real points of each pel
		std::vector<cv::Point3f> realpts;
		std::vector<cv::Point3f>::iterator it1;
		for( it1 = pts.begin(); it1 != pts.end(); it1++){
			float rx, ry;
	 		calc_real_coords(it1->z, it1->x*ALPHA, it1->y*ALPHA, rx, ry);
			float rz = it1->z;
			//std::cout << "Realpt (" << rx << "," << ry << "," << rz << ")";
			//std::cout << " from pt: (" << it1->x << "," << it1->y << "," << it1->z << ")" <<std::endl;
			realpts.push_back(cv::Point3f(rx,ry,rz));
		}

		//initialise random seed
		srand (time(NULL));

		//loop through each iteration
		cv::Point3f p0;
		cv::Point3f p1;
		cv::Point3f p2;
		cv::Point3f p1p0;
		cv::Point3f p2p0;
		cv::Point3f pnormal;

		for(int k = 0; k < RANSACK; k++){		
			//randomly choose 3 pixels in candidate
			int inliers = 0;
			bool print = false;

			int p0_index = rand() % size();
			p0 = realpts.at(p0_index);
		
			int p1_index = rand() % size();
			while((p1_index == p0_index)){
				p1_index = rand() % size(); //ensure that p1_index isnt equal to p0_index
			}
			p1 = realpts.at(p1_index);

			int p2_index = rand() % size();
			bool tryagain = true;
			while(tryagain){
				tryagain = false;
				p2_index = rand() % size(); //ensure that p2_index isnt equal to p0_index or p1_index
				if(p2_index == p1_index) tryagain = true;
				if(p2_index == p0_index) tryagain = true;
			}
			p2 = realpts.at(p2_index);

			//compute vectors in plane
			p1p0 = p1-p0;
			p2p0 = p2-p0;

			//compute plane normal
			pnormal = p1p0.cross(p2p0);
			float L2norm = norm(pnormal);
			if( L2norm !=0 ){
				pnormal = pnormal * (1/L2norm);
				float offset = -p0.dot(pnormal);
				if(isnan(pnormal.x)){
					print = true;
				}

				//now count how many inliers
				for( std::vector<cv::Point3f>::iterator it = realpts.begin(); it != realpts.end(); it++){
					float d = std::abs(pnormal.dot(*it) + offset);
					//std::cout << "Point: (" << it->x << "," << it->y << "," << it->z << "), distance: " << d << std::endl;
					if(d < EPSILON){
						inliers++;
					}
				}
				///*
				if(print){
					std::cout << "P0 = (" << p0.x << "," << p0.y << "," << p0.z << ") "<< std::endl;
					std::cout << "P1 = (" << p1.x << "," << p1.y << "," << p1.z << ") "<< std::endl;
					std::cout << "P2 = (" << p2.x << "," << p2.y << "," << p2.z << ") "<< std::endl;
					std::cout << "Plane = (" << pnormal.x << "," << pnormal.y << "," << pnormal.z << "," << offset << ") "<< std::endl;
					std::cout << "NaN's found in Candidate: " << id << " of size: " << size() << std::endl;
				}
				/*
				std::cout << "Plane = (" << pnormal.x << "," << pnormal.y << "," << pnormal.z << "," << offset << ") "<< std::endl;
				std::cout << "Candidate " << id << " size:  " << size() << std::endl;
				std::cout << "Candidate " << id << " inliers:  " << inliers << std::endl;
				//*/
				//and update max_inliers
				max_inlier_fraction = std::max(float(inliers)/size(), max_inlier_fraction);
			}
		} 

	} else {
		max_inlier_fraction = 1.0; 
	}

}

void candidate::calc_centre(){
	//TODO
	//find centre pixel from min and max pixel values
	int midx = 0.5*(xmin+xmax);
	int midy = 0.5*(ymin+ymax);
	float depth = depth_accumulator/size();

	//use true depth to convert real space location of candidate
	calc_real_coords(depth, midx*ALPHA, midy*ALPHA, centre.x, centre.y);
	centre.z = depth;

	/*//for now let us just use pel values
	centre[0] = pelx;
	centre[1] = pely;
	centre[2] = depth;*/

	//std::cout<<"Candidate: " << id << ", MU = (" << centre.z << "," << centre.y << "," << centre.z << ")" << std::endl;
	//std::cout << "xmin: " << xmin << ", xmax: " << xmax << ", ymin: " << ymin << ", ymax: " << ymax <<std::endl;
	
}

