/*
 * opencv_helper.h
 *
 *  Created on: 11.12.2014
 *      Author: fnolden
 */

#ifndef OPENCV_HELPER_H_
#define OPENCV_HELPER_H_

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ocl/ocl.hpp>


cv::Mat
correctGamma( cv::Mat& img, double gamma) {
	double inverse_gamma = 1.0 / gamma;

	cv::Mat lut_matrix(1, 256, CV_8UC1 );
	uchar * ptr = lut_matrix.ptr();
	for( int i = 0; i < 256; i++ )
	ptr[i] = (int)( pow( (double) i / 255.0, inverse_gamma ) * 255.0 );

	cv::Mat result;
	LUT( img, lut_matrix, result );

	return result;
}

cv::Mat
clahe(cv::Mat& img, double cliplimit) {
	// READ RGB color image and convert it to Lab
	cv::Mat lab_image;
	cv::cvtColor(img, lab_image, CV_BGR2Lab);
	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]
	cv::Mat dst;
	// apply the CLAHE algorithm to the L channel
	cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
	clahe->setClipLimit(cliplimit);
	clahe->apply(lab_planes[0], dst);
	// Merge the the color planes back into an Lab image
	dst.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);
	// convert back to RGB
	cv::cvtColor(lab_image, img, CV_Lab2BGR);
	return img;
}

cv::Mat
brightness(cv::Mat& img, double alpha, double beta) {
	// READ RGB color image and convert it to Lab
	cv::Mat lab_image;
	cv::cvtColor(img, lab_image, CV_BGR2Lab);
	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes); // now we have the L image in lab_planes[0]
	cv::Mat dst;
	// apply the CLAHE algorithm to the L channel
	lab_planes[0].convertTo(dst, -1, alpha, beta );
	dst.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);
	// convert back to RGB
	cv::cvtColor(lab_image, img, CV_Lab2BGR);
	return img;
}



#endif /* OPENCV_HELPER_H_ */
