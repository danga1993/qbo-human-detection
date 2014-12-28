#include <vector>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "feature/featurevector.h"
#include "hogvector.h"
#include "config.h"

void HOGVector::set_candidate(const candidate& cand)
{
	cv:Size cand_size(cand->im.size());
	int cell_count = HOG_HOR_CELL_COUNT * HOG_VER_CELL_COUNT;
	int cell_width = cand_size.width / HOG_HOR_CELL_COUNT;
	int cell_height = cand_size.height / HOG_VER_CELL_COUNT;
	
	std::vector<cv::Mat> raw_cells(cell_count);
	std::vector<cv::Mat> diff_hor_cells(cell_count);
	std::vector<cv::Mat> diff_ver_cells(cell_count);
	std::vector<cv::Mat> cell_angles_all(cell_count);
	int[] bin_upper_limits = new int[HOG_BIN_COUNT];
	std::vector<cv::Mat> cell_hists_all(cell_count);
	
	// split the candidate into cells
	for (int n = 0; n < cell_count; n++)
	{
		int x = cell_count * n;
		int y = cell_count * n;
		cv::Mat raw_cell = cand(x, y, cell_width, cell_height);
		
		raw_cells.push_back(cell);
	}
	
	// differentiate horizontally
	for (int n = 0; n < cell_count; n++)
	{
		cv::Mat diff_hor_cell = cv::zeros(cell_height, cell_width - 2, CV_32FC1);
		
		for (int x = 1; x < cell_width-1; x++)
			for (int y = 0; y < cell_height; y++)
				diff_hor_cell.at(x-1, y) = raw_cells.at(n).at(x+1, y) - raw_cells.at(n).at(x-1, y);
		
		diff_hor_cells.push_back(diff_hor_cell);
	}
	
	// differentiate vertically
	for (int n = 0; n < cell_count; n++)
	{
		cv::Mat diff_ver_cell = cv::zeros(cell_height - 2, cell_width, CV_32FC1);
		
		for (int x = 0; x < cell_width; x++)
			for (int y = 1; y < cell_height-1; y++)
				diff_ver_cell.at(x, y-1) = raw_cells.at(n).at(x, y+1) - raw_cells.at(n).at(x, y-1);
		
		diff_ver_cells.push_back(diff_ver_cell);
	}
	
	// angle of each pixel
	for (int n = 0; n < cell_count; n++)
	{
		cv::Mat cell_angles = cv::zeros(cand_size.height - 2, cand_size.width - 2, CV_32FC1);
	
		for (int x = 0; x < cell_width-2; x++)
			for (int y = 0; y < cell_height-2; y++)
				cell_angles.at(x, y) = diff_ver_cell.at(n).at(x, y) / diff_hor_cell.at(n).at(x, y);
		
		cell_angles_all.push_back(angle_cell);
	}
	
	// get ranges of histogram bins
	for (int m = 1; m <= HOG_BIN_COUNT; m++)
		bin_upper_limits[m-1] = (180.0 / HOG_BIN_COUNT) * m;
	
	// histogram of gradients for each cell
	for (int n = 0; n < cell_count; n++)
	{
		cv::Mat cell_hist = cv::zeros(HOG_BIN_COUNT);
		
		for (int x = 0; x < cell_width-2; x++)
			for (int y = 0; y < cell_height-2; y++)
				for (int m = 0; m < HOG_BIN_COUNT; m++)
					if (cell_angles_all.at(x, y) < bin_upper_limits[m])
					{
						cell_hist.at(m)++;
						break;
					}
		
		cell_hists_all.push_back(cell_hist);
	}
}

void getfeatures(std::vector<int> feature_ids, cv::Mat& features)
{
	features = cv::zeros(feature_ids.size());

	for (int i = 0; i < feature_ids.size(); i++)
	{
		int id = feature_ids.at(i);
		features.
	}
}