#ifndef IMAGE_CLOUD_COMMON_PIPELINE_IMAGE_H_
#define IMAGE_CLOUD_COMMON_PIPELINE_IMAGE_H_

#include <assert.h>
#include <opencv2/opencv.hpp>

namespace image_cloud{
	inline void convert_to_grey(const cv::Mat &in_unknown, cv::Mat &grey){
		// Convert Greyscale
		if (in_unknown.channels() != 1) {
			cvtColor(in_unknown, grey, CV_BGR2GRAY);
			assert(grey.channels() == 1);
		} else {
			in_unknown.copyTo(grey);
		}
	}

	inline void create_empty_grey_mat(cv::Mat &image, int rows, int cols){
		image.create(rows, cols, CV_8U);
	}

	/**	Create inverse transformed image
	 * 	Grayscale the image
	 * 	Edge Filter
	 * 	Inverse distance transformation
	 */
	inline void create_inverse_transformed(const cv::Mat& in, cv::Mat &out){
		cv::Mat grey,edge;

		create_empty_grey_mat(edge, in.rows, in.cols);
		create_empty_grey_mat(out, in.rows, in.cols);

		// Convert Greyscale
		convert_to_grey(in, grey);

		// Edge filter
		filter_2d::edge_max<uchar>( grey, edge);

		// Inverse distance transform
		filter_2d::inverse_distance_transformation<uchar,uchar>(edge, out);
	}
}
#endif
