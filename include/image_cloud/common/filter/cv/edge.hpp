#include <image_cloud/common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>

#ifndef FILTER2D_EDGE_H_
#define FILTER2D_EDGE_H_

namespace filter_2d
{


inline
void get_diff(int x, int y, int cx, int cy, int &result, cv::Mat& in) {
	if (between(-1, x, in.rows) && between(-1, y, in.cols)) {
		// hit upper left
		result = std::max(abs(in.at<uchar>(x, y) - in.at<uchar>(cx, cy)),	result);
	}
}

inline
int max_diff(int cx, int cy, cv::Mat &in){
	//Check
	int result = 0;

	get_diff(cx -1, cy -1, cx, cy, result, in);
	get_diff(cx	  , cy -1, cx, cy, result, in);
	get_diff(cx +1, cy -1, cx, cy, result, in);
	get_diff(cx -1, cy   , cx, cy, result, in);
	get_diff(cx +1, cy   , cx, cy, result, in);
	get_diff(cx -1, cy +1, cx, cy, result, in);
	get_diff(cx   , cy +1, cx, cy, result, in);
	get_diff(cx +1, cy +1, cx, cy, result, in);

	return result;
}

void
edge_max(cv::Mat &in, cv::Mat &out)
{
		for(int y = 0; y < in.cols; y++)
		{
			for(int x = 0; x < in.rows; x++)
			{
				out.at<uchar>(x,y) = max_diff(x, y, in);
			}
		}
}


}

#endif
