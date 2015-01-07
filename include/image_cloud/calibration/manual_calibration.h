/*
 * image_pointcloud_nodelet.h
 *
 *  Created on: 11.08.2014
 *      Author: fnolden
 */

#ifndef CALIBRATION_MANUAL_NODELET_H_
#define CALIBRATION_MANUAL_NODELET_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

//#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <dynamic_reconfigure/server.h>
#include <image_cloud/manual_calibrationConfig.h>
//#include "nodelet/nodelet.h"


namespace image_cloud {

class Manual_calibration {
	typedef image_cloud::manual_calibrationConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	virtual void onInit();
	virtual ~Manual_calibration();
	virtual void loop();
	virtual void reconfiguration_callback(Config &config, uint32_t level);
	virtual void initParams();
	virtual void params();
private:
	ros::NodeHandle nh;
	//ros::Publisher pub;
	std::string node_name_;
	//std::string pub_topic_;

	//tf2_ros::Buffer tfBuffer;
	//boost::shared_ptr<tf2_ros::TransformListener> tfListener;
	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	//tf2_ros::Transform transform;

	Config config_;
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	//sensor_msgs::JointState msg_;
};
} /* end namespace */

#endif /* IMAGE_CLOUD_NODELET_H_ */
