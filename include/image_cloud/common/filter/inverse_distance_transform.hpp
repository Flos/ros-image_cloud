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

template <typename Type_in, typename Type_out>
inline float calc(float &val, const float &psi, int x, int y, const cv::Mat& in, cv::Mat& out) {
	val = val * psi;
	if (in.at<Type_in>(x, y) > val)
	{
		val = in.at<Type_in>(x, y);
	}
	else
	{
		if (out.at<Type_out>(x, y) < val)
		{
			out.at<Type_out>(x, y) = val;
		}
		else{
			val = out.at<Type_out>(x, y);
		}
	}
	return val;
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int y = 0; y < in.cols; ++y)
	{
		val = 0;
		for(int x = 0; x < in.rows; ++x)
		{
			val = calc<Type_in, Type_out>(val, psi, x, y, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int y = in.cols-1; y >= 0; --y)
	{
		val = 0;
		for(int x = in.rows -1; x >= 0; --x)
		{
			val = calc<Type_in, Type_out>(val, psi, x, y, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int x = 0; x < in.rows; ++x)
	{
		val = 0;
		for(int y = 0; y < in.cols; ++y)
		{
			val = calc<Type_in, Type_out>(val, psi, x, y, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int x = in.rows -1; x >= 0; --x)
	{
		val = 0;
		for(int y = in.cols -1; y >= 0; --y)
		{
			val = calc<Type_in, Type_out>(val, psi, x, y, in, out);
		}
	}
}


template <typename Type_in, typename Type_out>
void
inverse_distance_transformation(cv::Mat &in, cv::Mat &out, float alpha = 0.333333333, float psi = 0.98)
{
		assert(in.channels() == 1);
		assert(in.depth() == CV_8U);

		assert(in.size == out.size);

		out = cv::Mat(cv::Size(in.cols, in.rows), cv::DataType<Type_out>::type, cv::Scalar(0));

		neighbors_x_pos<Type_in, Type_out>(in, out, psi, alpha);
		neighbors_x_neg<Type_in, Type_out>(in, out, psi, alpha);
		neighbors_y_pos<Type_in, Type_out>(in, out, psi, alpha);
		neighbors_y_neg<Type_in, Type_out>(in, out, psi, alpha);

		for(int y = 0; y < in.cols; y++)
		{
			for(int x = 0; x < in.rows; x++)
			{
				int val = alpha * in.at<Type_in>(x,y) + (1 - alpha)*(float)out.at<Type_out>(x,y);
				out.at<Type_out>(x,y) = val;
			}
		}
		//printf("6\n");
}


}
#endif
