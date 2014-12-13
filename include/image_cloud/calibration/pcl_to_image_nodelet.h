/*
 * image_pointcloud_nodelet.h
 *
 *  Created on: 11.08.2014
 *      Author: fnolden
 */

#ifndef PCL_TO_IMAGE_NODELET_H_
#define PCL_TO_IMAGE_NODELET_H_

#include "ros/ros.h"
#include <sstream>
#include <string.h>
#include <map>
#include <iostream>
#include <functional>

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include "nodelet/nodelet.h"

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>
#include <pcl/filters/frustum_culling.h>

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

namespace image_cloud {

typedef sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, PointCloud> Image_to_cloud_sync;


class Pcl_to_image_nodelet : public nodelet::Nodelet {
public:
	virtual void onInit();
	virtual ~Pcl_to_image_nodelet();
	virtual void callback(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud);

private:
	message_filters::Subscriber<sensor_msgs::CameraInfo>* image_info_sub;
	message_filters::Subscriber<PointCloud>* pointcloud_sub;
	message_filters::Synchronizer<Image_to_cloud_sync>* sync;
	ros::Subscriber sub_;

	image_transport::ImageTransport *it_;
	image_transport::Publisher pub_;

	std::string node_name_;
	std::string subscribe_topic_pcl_;
	std::string subscribe_topic_img_info_;
	std::string filter_;
	std::string image_frame_id_;
	std::string reference_frame_id_;
	std::string publish_image_topic_;
	double resize_faktor_x_;
	double resize_faktor_y_;

	ros::NodeHandle nh;

	image_geometry::PinholeCameraModel camera_model;

	tf::TransformListener listener_pointcloud_transform;

};
} /* end namespace */

#endif /* IMAGE_CLOUD_NODELET_H_ */
