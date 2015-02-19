#include <image_cloud/common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

#ifndef FILTER2D_EDGE_H_
#define FILTER2D_EDGE_H_

namespace filter_2d
{


template <typename ImageT>
inline
void get_diff(int col, int row, int col_c, int row_c, int &result, cv::Mat& in) {
	if (between(-1, col, in.cols) && between(-1, row, in.rows)) {
		// hit upper left
		result = std::max(abs(in.at<ImageT>(row, col) - in.at<ImageT>(row_c, col_c)),	result);
	}
}

template <typename ImageT>
inline
int max_diff_neighbors(int row_c, int col_c, cv::Mat &in){
	//Check
	int result = 0;

	get_diff<ImageT>(col_c -1, row_c -1, col_c, row_c, result, in);
	get_diff<ImageT>(col_c	  , row_c -1, col_c, row_c, result, in);
	get_diff<ImageT>(col_c +1, row_c -1, col_c, row_c, result, in);
	get_diff<ImageT>(col_c -1, row_c   , col_c, row_c, result, in);
	get_diff<ImageT>(col_c +1, row_c   , col_c, row_c, result, in);
	get_diff<ImageT>(col_c -1, row_c +1, col_c, row_c, result, in);
	get_diff<ImageT>(col_c   , row_c +1, col_c, row_c, result, in);
	get_diff<ImageT>(col_c +1, row_c +1, col_c, row_c, result, in);

	return result;
}

template <typename ImageT>
inline void
edge_max(cv::Mat &in, cv::Mat &out)
{

	//printf("rows: %d, cols: %d\n", in.rows, in.cols);
	//printf("rows: %d, cols: %d\n", out.rows, out.cols);
	assert(in.rows == out.rows);
	assert(in.cols == out.cols);
	assert(in.depth() == out.depth());
	assert(in.channels() == out.channels());

	for(int r = 0; r < in.rows; r++)
	{
		for(int c = 0; c < in.cols; c++)
		{
			out.at<ImageT>(r,c) = max_diff_neighbors<ImageT>(r, c, in);
		}
	}
}


}

#endif
