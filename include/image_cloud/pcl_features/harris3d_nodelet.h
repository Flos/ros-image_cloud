/*
 * Edgedetectornodelet.h
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#ifndef HARRIS3DEDGEDETECTORNODELET_H_
#define HARRIS3DEDGEDETECTORNODELET_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/keypoints/harris_3d.h>

#include <eigen3/Eigen/Core>

#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include <image_cloud/harris3dConfig.h>

namespace image_cloud {

class Harris3d_nodelet : public nodelet::Nodelet {
	typedef sensor_msgs::PointCloud2 PointCloud;
	typedef image_cloud::harris3dConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	Harris3d_nodelet();
	virtual void onInit();
	virtual void callback(const PointCloud::ConstPtr &input_msg_cloud_ptr);
	virtual void reconfigure_callback(Config &config, uint32_t level);
	virtual ~Harris3d_nodelet();
private:
	pcl::HarrisKeypoint3D<pcl::PointXYZI, pcl::PointXYZI, pcl::PointNormal> detector;
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> ne;

	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	Config config_;

	std::string node_name_;

};

} /* namespace ladybug */

#endif /* HARRIS3DEDGEDETECTORNODELET_H_ */
