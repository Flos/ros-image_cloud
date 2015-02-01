/*
 * auto_calibration.h
 *
 *  Created on: 29.01.2015
 *      Author: fnolden
 */
#ifndef SRC_CALIBRATION_AUTO_CALIBRATION_H_
#define SRC_CALIBRATION_AUTO_CALIBRATION_H_

#include <ros/ros.h>
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>


#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

// Own hpp
#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/pointcloud_rgb.hpp>
#include <image_cloud/common/filter/cv/inverse_distance_transform.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>
#include <image_cloud/common/calibration/score.hpp>
#include <image_cloud/common/calibration/structs.hpp>
#include <image_cloud/common/calibration/grid_search.hpp>
#include <image_cloud/common/calibration/transform_publisher.hpp>
#include <image_cloud/common/calibration/pipeline/image.hpp>
#include <image_cloud/common/calibration/pipeline/pointcloud.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_cloud/continuous_calibrationConfig.h>


namespace image_cloud {

class Continuous_calibration {

	typedef sensor_msgs::PointCloud2 PointCloud2_msg;
	typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, PointCloud2_msg> Sync_filter;
	typedef image_cloud::continuous_calibrationConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;


	struct Temp_data{
		search::Window window;
		image_geometry::PinholeCameraModel camera_model;
		bool is_camera_model_set;
		tf::Transform calibration;
	};

public:
	Continuous_calibration();
	Continuous_calibration(ros::NodeHandle& nh, ros::NodeHandle &nh_private);
	virtual ~Continuous_calibration();
	virtual void callback_info(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info);
	virtual void callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
	virtual void reconfigure_callback(Config &config, uint32_t level);
	virtual void init(ros::NodeHandle& nh, ros::NodeHandle &nh_private);
	virtual void init_params();
	virtual void init_sub();
	virtual void init_pub();
	virtual void print_params();
private:
	boost::shared_ptr<ReconfigureServer> reconfigure_server;

	boost::shared_ptr<message_filters::Subscriber<PointCloud2_msg> > sub_pointcloud;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > sub_image;
	boost::shared_ptr<message_filters::Synchronizer<Sync_filter> > sync_filter;

	ros::Subscriber sub_info;

	ros::Publisher pub_cloud;
	ros::Publisher pub_cloud_color;
	Transform_publisher pub_tf;

	boost::mutex lock_callback;
	Config config;
	Temp_data data;

	ros::NodeHandle nh;
	ros::NodeHandle nh_private;

	boost::shared_ptr<tf::TransformListener> tf_listener;
};

} /* namespace image_cloud */

#endif /* SRC_CALIBRATION_AUTO_CALIBRATION_H_ */
