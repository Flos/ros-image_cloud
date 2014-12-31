/*
 * Edgedetectornodelet.h
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#ifndef DILATENODELET_H_
#define DILATENODELET_H_

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

#include <dynamic_reconfigure/server.h>
#include <image_cloud/dilateConfig.h>

namespace image_cloud {

class Dilate : public nodelet::Nodelet {
	typedef image_cloud::dilateConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	Dilate();
	virtual void onInit();
	virtual void callback(const sensor_msgs::ImageConstPtr& input_msg_image);
	virtual void reconfigure_callback(Config &config, uint32_t level);
	virtual ~Dilate();
private:
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;


	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	Config config_;

	cv::Mat element;

	std::string node_name_;

	ros::NodeHandle nh;

	void reset_image_transport();
};

} /* namespace ladybug */

#endif /* EDGEDETECTORNODELET_H_ */
