#ifndef IMAGE_CLOUD_COMMON_PIPELINE_IMAGE_H_
#define IMAGE_CLOUD_COMMON_PIPELINE_IMAGE_H_

#include <assert.h>
#include <opencv2/opencv.hpp>

#include <image_cloud/common/calibration/pipeline/enums.h>

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
		image = cv::Mat::zeros(rows, cols, CV_8U);
	}

	/**	Create inverse transformed image
	 * 	Grayscale the image
	 * 	Edge Filter
	 * 	Inverse distance transformation
	 */
	inline void create_inverse_transformed(const cv::Mat& in, cv::Mat &out, image_filter::edge::Edge filter = image_filter::edge::MAX ){
		cv::Mat grey,edge;

		create_empty_grey_mat(edge, in.rows, in.cols);
		create_empty_grey_mat(out, in.rows, in.cols);

		// Convert Greyscale
		convert_to_grey(in, grey);

		// Edge filter
		switch (filter) {
			default:
			case image_filter::edge::OFF:
				grey.copyTo(edge);
				break;
			case image_filter::edge::CANNY:
				cv::Canny( grey, edge, 30, 100, 3, 	false );
			break;
			case image_filter::edge::LAPLACE:
				cv::Laplacian( grey, edge, 3, 1, 0 );
			break;
			case image_filter::edge::MAX:
				filter_2d::edge_max<uchar>( grey, edge);
				break;

		}

		// Inverse distance transform
		filter_2d::inverse_distance_transformation<uchar,uchar>(edge, out);
	}

	inline void create_blured_image(const cv::Mat& in, cv::Mat &out, image_filter::blur::Blur filter = image_filter::blur::BLUR){
		switch(filter){
			default:
			case ::image_filter::blur::OFF:
				in.copyTo(out);
				break;
			case ::image_filter::blur::BILATERAL:
				cv::bilateralFilter( in, out, 2, 2*2, 1);
				break;
			case ::image_filter::blur::BLUR:
				cv::blur( in, out, cv::Size(2,2), cv::Point(-1,-1) );
				break;
			case ::image_filter::blur::GAUSSIAN:
				cv::GaussianBlur( in, out, cv::Size(2,2), 0, 0);
				break;
			case ::image_filter::blur::MEDIAN:
				cv::medianBlur( in, out, 2);
				break;
		}
	}
}
#endif
