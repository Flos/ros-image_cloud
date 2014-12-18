/*
 * image_pointcloud_nodelet.h
 *
 *  Created on: 11.08.2014
 *      Author: fnolden
 */

#ifndef IMAGE_BLUR_NODELET_H_
#define IMAGE_BLUR_NODELET_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"

#include "opencv_helper.hpp"
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Core>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace image_cloud {

class Blur : public nodelet::Nodelet {
public:
	virtual void onInit();
	virtual ~Blur();
	virtual void callback(const sensor_msgs::ImageConstPtr& input_msg_image);

private:
	image_transport::ImageTransport *it_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;

	std::string node_name_;
	std::string subscribe_topic_;
	std::string publish_topic_;
	int filter_;
	int kernel_size_;

	ros::NodeHandle nh;
};
} /* end namespace */

#endif /* IMAGE_BLUR_NODELET_H_ */
