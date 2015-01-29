#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl_ros/transforms.h>
#include <tf/tf.h>

#ifndef MY_TRANSFORM_H_
#define MY_TRANSFORM_H_

namespace image_cloud
{

template <typename PointT>
bool inline transform_pointcloud(tf::TransformListener &transform_listener, pcl::PointCloud<PointT> &cloud, std::string target_frame, ros::Time target_time, std::string source_frame, ros::Time source_time, std::string tf_error ){

	if( target_frame != source_frame || target_time != source_time )
	{
		if(!transform_listener.waitForTransform(target_frame.c_str(), 	// target frame
													source_frame.c_str(), 	// source frame
													target_time, 			// target time
													ros::Duration(5.0),		// block time out
													ros::Duration(0.1),		// poll interval
															&tf_error		// error string
															)
		)
		{
			return false;
		}

		if (!pcl_ros::transformPointCloud(target_frame.c_str(), cloud, cloud, transform_listener)) {
			 tf_error = "pcl_ros: Cannot transform PointCloud from [" + source_frame + "] to [" + target_frame + "]";
			 return false;
		}
	}

	return true;
}


template <typename PointT>
void inline create_transform(tf::Transform &tf, float tx, float ty, float tz, float roll, float pitch, float yaw){
   tf::Quaternion q;
   q.setRPY(roll, pitch, yaw);

   tf.setOrigin(tf::Vector3( tx, ty, tz));
   tf.setRotation(q);
}

template <typename PointT>
void inline transform_pointcloud(const pcl::PointCloud<PointT> &in, pcl::PointCloud<PointT> &out, tf::Transform tf){
   pcl_ros::transformPointCloud(in, out, tf);
}

template <typename PointT>
void inline transform_pointcloud(pcl::PointCloud<PointT> &cloud, tf::Transform tf){
	transform_pointcloud<PointT>(cloud, cloud, tf);
}

template <typename PointT>
void inline transform_pointcloud(const pcl::PointCloud<PointT> &in, pcl::PointCloud<PointT> &out, float tx, float ty, float tz, float roll, float pitch, float yaw){
   tf::Transform tf;
   create_transform<PointT>(tf, tx, ty, tz, roll, pitch, yaw);

   transform_pointcloud(in, out, tf);
}

template <typename PointT>
void inline transform_pointcloud(pcl::PointCloud<PointT> &cloud, float tx, float ty, float tz, float roll, float pitch, float yaw){
   transform_pointcloud(cloud, cloud, tx, ty, tz, roll, pitch, yaw);
}

}
#endif /* MY_TRANSFORM_H_ */
