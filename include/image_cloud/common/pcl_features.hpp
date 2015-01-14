#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/conversions.h>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/keypoints/harris_3d.h>

#ifndef PCL_FEATURES_H_
#define PCL_FEATURES_H_

inline pcl::PointCloud<pcl::PointXYZINormal>::Ptr
calculate_normals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
		double normal_search_radius = 0.05
)
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n(
			new pcl::search::KdTree<pcl::PointXYZI>());
	// Estimate the normals of the cloud_xyz
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree_n);
	ne.setRadiusSearch(normal_search_radius);
	ne.compute(*cloud_normals);
	return cloud_normals;
}

inline void
extract_intensity(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
		cv_bridge::CvImage& image_pcl,
		cv_bridge::CvImage& image_depth,
		int point_size = 1
)
{
	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZI& pt, (*cloud).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, point_size, cv::Scalar(pt.intensity), -1);
				cv::circle(image_depth.image, point_image, point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

inline void
extract_normals(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
		cv_bridge::CvImage& image_pcl,
		cv_bridge::CvImage& image_depth,
		int point_size = 1,
		double normal_search_radius = 0.05
)
{
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals = calculate_normals(cloud, normal_search_radius);
	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZINormal& pt, (*cloud_normals).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Colors
				cv::circle(image_pcl.image, point_image, point_size, cv::Scalar(pt.data_n[0]/3 + pt.data_n[1]/3 + pt.data_n[2]/3 ), -1);
				cv::circle(image_depth.image, point_image, point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

inline void
extract_intensity_and_normals(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
		cv_bridge::CvImage& image_pcl,
		cv_bridge::CvImage& image_depth,
		int point_size = 1,
		double normal_search_radius = 0.05
) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals = calculate_normals(cloud, normal_search_radius);

	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZINormal& pt, (*cloud_normals).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, point_size, cv::Scalar(pt.intensity/2 + (pt.data_n[0]/3 + pt.data_n[1]/3 + pt.data_n[2]/3)/2), -1);
				cv::circle(image_depth.image, point_image, point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

#endif
