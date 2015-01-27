#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>


#ifndef FILTER2D_INVERSE_DISTANCE_TRANSFORMATION_H_
#define FILTER2D_INVERSE_DISTANCE_TRANSFORMATION_H_

namespace filter_2d
{

float
max_edge_neighbors(int i, int j, float psi, cv::Mat &image){
	float result = 0;

	for(int y = 0; y < image.cols; y++)
	{
		for(int x = 0; x < image.rows; x++)
		{
			 float value =  image.at<char>(x,y) * pow(psi, std::max( abs(x - i), abs( y - j)));
			 result = std::max( result, value );
		}
	}
	printf("Result: %f\n", result);
	return result;
}


inline float calc(int &val, const float &psi, int x, int y, const cv::Mat& in, cv::Mat& out) {
	val = val * psi;
	if (in.at<uchar>(x, y) > val)
	{
		val = in.at<uchar>(x, y);
	}
	else
	{
		if (out.at<uchar>(x, y) < val)
		{
			out.at<uchar>(x, y) = val;
		}
		else{
			val = out.at<uchar>(x, y);
		}
	}
	return val;
}

inline void
neighbors_x_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	int val = 0;

	for(int y = 0; y < in.cols; ++y)
	{
		for(int x = 0; x < in.rows; ++x)
		{
			val = calc(val, psi, x, y, in, out);
		}
	}
}

inline void
neighbors_x_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	int val = 0;

	for(int y = in.cols-1; y >= 0; --y)
	{
		for(int x = in.rows -1; x >= 0; --x)
		{
			val = calc(val, psi, x, y, in, out);
		}
	}
}

inline void
neighbors_y_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	int val = 0;

	for(int x = 0; x < in.rows; ++x)
	{
		for(int y = 0; y < in.cols; ++y)
		{
			val = calc(val, psi, x, y, in, out);
		}
	}
}

inline void
neighbors_y_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	int val = 0;

	for(int x = in.rows -1; x >= 0; --x)
	{
		for(int y = in.cols -1; y >= 0; --y)
		{
			val = calc(val, psi, x, y, in, out);
		}
	}
}


void
inverse_distance_transformation(cv::Mat &in, cv::Mat &out, float alpha = 0.333333333, float psi = 0.98)
{
		assert(in.channels() == 1);
		assert(in.depth() == CV_8U);

		assert(in.size == out.size);

		out = cv::Mat(cv::Size(in.cols, in.rows), CV_8UC1, cv::Scalar(0));

		neighbors_x_pos(in, out, psi, alpha);
		//printf("2\n");
		neighbors_x_neg(in, out, psi, alpha);
		//printf("3\n");
		neighbors_y_pos(in, out, psi, alpha);
		//printf("4\n");
		neighbors_y_neg(in, out, psi, alpha);
		//printf("5\n");

		for(int y = 0; y < in.cols; y++)
		{
			for(int x = 0; x < in.rows; x++)
			{
				//printf("values: in %d, out %d", in.at<uchar>(x,y), (int)((1 - alpha)* out.at<uchar>(x,y)));
				out.at<uchar>(x,y) = alpha * ((int)in.at<uchar>(x,y)) + (1 - alpha)* out.at<uchar>(x,y);
				//printf(",result: %d \n", out.at<uchar>(x,y));

			}
		}
		//printf("6\n");
}


}
#endif
