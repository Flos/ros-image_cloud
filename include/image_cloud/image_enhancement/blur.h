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

#include <dynamic_reconfigure/server.h>
#include <image_cloud/blurConfig.h>

namespace image_cloud {

class Blur : public nodelet::Nodelet {
	typedef image_cloud::blurConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	virtual void onInit();
	virtual ~Blur();
	virtual void callback(const sensor_msgs::ImageConstPtr& input_msg_image);
	virtual void reconfigure_callback(Config &config, uint32_t level);

private:
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;


	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	Config config_;

	std::string node_name_;
	ros::NodeHandle nh;
};
} /* end namespace */

#endif /* IMAGE_BLUR_NODELET_H_ */
