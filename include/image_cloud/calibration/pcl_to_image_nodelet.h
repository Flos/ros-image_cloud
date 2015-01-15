/*
 * image_pointcloud_nodelet.h
 *
 *  Created on: 11.08.2014
 *      Author: fnolden
 */

#ifndef PCL_TO_IMAGE_NODELET_H_
#define PCL_TO_IMAGE_NODELET_H_

#include <ros/ros.h>
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

#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>

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
#include <pcl/keypoints/harris_3d.h>
#include <dynamic_reconfigure/server.h>
#include <image_cloud/pcl_imageConfig.h>


namespace image_cloud {

typedef sensor_msgs::PointCloud2 PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudColor;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, PointCloud> Image_to_cloud_sync;


class Pcl_to_image_nodelet : public nodelet::Nodelet {
	typedef image_cloud::pcl_imageConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
public:
	virtual void onInit();
	virtual ~Pcl_to_image_nodelet();
	virtual void callback(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud);
	virtual void reconfigure_callback(Config &config, uint32_t level);
	virtual void init_params();
	virtual void init_sub();
	virtual void init_pub();
	virtual void params();
private:
	boost::shared_ptr<ReconfigureServer> reconfigure_server_;
	boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > image_info_sub;
	boost::shared_ptr<message_filters::Subscriber<PointCloud> > pointcloud_sub;
	boost::shared_ptr<message_filters::Synchronizer<Image_to_cloud_sync> > sync;


	boost::shared_ptr<image_transport::ImageTransport> it_;
	ros::Subscriber sub_;
	image_transport::Publisher pub_;
	image_transport::Publisher pub_depth_;
	ros::Publisher pub_cloud_;
	//image_transport::Publisher pub_range_;

	std::string node_name_;
	boost::mutex config_lock_;
	Config config_;
	ros::NodeHandle nh;

	image_geometry::PinholeCameraModel camera_model;

	boost::shared_ptr<tf::TransformListener> listener_pointcloud_transform;
};
} /* end namespace */

#endif /* IMAGE_CLOUD_NODELET_H_ */
