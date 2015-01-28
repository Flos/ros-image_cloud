/*
 * Edgedetectornodelet.h
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#ifndef EDGEDETECTORNODELET_H_
#define EDGEDETECTORNODELET_H_

#include <ros/ros.h>
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <dynamic_reconfigure/server.h>
#include <image_cloud/edge_detectorConfig.h>

namespace image_cloud {

class Edge_detector_nodelet : public nodelet::Nodelet {
public:
	Edge_detector_nodelet();
	virtual void onInit();
	virtual void callback(const sensor_msgs::ImageConstPtr& input_msg_image);
	virtual void reconfigure_callback(image_cloud::edge_detectorConfig &config, uint32_t level);
	virtual ~Edge_detector_nodelet();
private:
	boost::shared_ptr<image_transport::ImageTransport> it_;
	image_transport::Subscriber sub_;
	image_transport::Publisher pub_;

	typedef image_cloud::edge_detectorConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	Config config_;

	std::string node_name_;

	ros::NodeHandle nh;
};

} /* namespace ladybug */

#endif /* EDGEDETECTORNODELET_H_ */
