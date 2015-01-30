#include <image_cloud/common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <math.h>


#ifndef FILTER2D_INVERSE_DISTANCE_TRANSFORMATION_H_
#define FILTER2D_INVERSE_DISTANCE_TRANSFORMATION_H_

namespace filter_2d
{

inline float
max_edge_neighbors(int i, int j, float psi, cv::Mat &image){
	float result = 0;

	for(int row = 0; row < image.rows; ++row)
	{
		for(int col = 0; col < image.cols; ++col)
		{
			 float value =  image.at<char>(row, col) * pow(psi, std::max( abs(col - i), abs( row - j)));
			 result = std::max( result, value );
		}
	}
	printf("Result: %f\n", result);
	return result;
}

template <typename Type_in, typename Type_out>
inline float calc(float &val, const float &psi, int row, int col, const cv::Mat& in, cv::Mat& out) {

	val = val * psi; /* Fade out the value */

	if (in.at<Type_in>(row, col) > val) /* In Value in the image bigger than the current value */
	{
		val = in.at<Type_in>(row, col); /* yes, get the bigger value */
	}

	if (out.at<Type_out>(row, col) < val) /* is the calculated value bigger then the value in the filtered image? */
	{
		out.at<Type_out>(row, col) = val; /* yes, store the calculated value in the filtered image */
	}
	else{
		val = out.at<Type_out>(row, col); /* no, the value of the filtered image is bigger use it */
	}

	return val;
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int row = 0; row < in.rows; ++row)
	{
		val = 0;
		for(int col = 0; col < in.cols; ++col)
		{
			val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_x_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int row = 0; row < in.rows; ++row)
	{
		val = 0;
		for(int col = in.cols -1; col >= 0; --col)
		{
			val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_pos(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int col = 0; col < in.cols; ++col)
	{
		val = 0;
		for(int row = 0; row < in.rows; ++row)
		{
			val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
		}
	}
}

template <typename Type_in, typename Type_out>
inline void
neighbors_y_neg(cv::Mat &in, cv::Mat &out, float psi, float alpha){
	float val = 0;

	for(int col = 0;  col < in.cols; ++col)
	{
		val = 0;
		for(int row = in.rows -1; row >= 0; --row)
		{
			val = calc<Type_in, Type_out>(val, psi, row, col, in, out);
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
	assert(in.rows == out.rows);
	assert(in.cols == out.cols);

	neighbors_x_pos<Type_in, Type_out>(in, out, psi, alpha);
	neighbors_x_neg<Type_in, Type_out>(in, out, psi, alpha);
	neighbors_y_pos<Type_in, Type_out>(in, out, psi, alpha);
	neighbors_y_neg<Type_in, Type_out>(in, out, psi, alpha);

	for(int row = 0; row < in.rows; row++)
	{
		for(int col = 0; col < in.cols; col++)
		{
			int val = alpha * in.at<Type_in>(row,col) + (1 - alpha)*(float)out.at<Type_out>(row,col);
			out.at<Type_out>(row, col) = val;
		}
	}
}


}
#endif
