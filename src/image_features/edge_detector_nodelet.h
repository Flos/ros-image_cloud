/*
 * Edgedetectornodelet.h
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#ifndef EDGEDETECTORNODELET_H_
#define EDGEDETECTORNODELET_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace image_cloud {

class Edge_detector_nodelet : public nodelet::Nodelet {
public:
	Edge_detector_nodelet();
	virtual void onInit();
	virtual void callback(const sensor_msgs::ImageConstPtr& input_msg_image);
	virtual ~Edge_detector_nodelet();
private:
	image_transport::ImageTransport *it_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;

	std::string node_name_;
	std::string subscribe_topic_;
	std::string publish_topic_;
	int filter_;
	int kernel_size_;
	int threshold1_;
	int threshold2_;
	bool publish_color_;

	ros::NodeHandle nh;
};

} /* namespace ladybug */

#endif /* EDGEDETECTORNODELET_H_ */
