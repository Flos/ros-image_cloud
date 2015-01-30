#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>

#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

#ifndef POINTCLOUD_COLOR_2D_H_
#define POINTCLOUD_COLOR_2D_H_

namespace image_cloud{

template <typename PointT, typename Image_element_T>
inline void
pointcloud_rgb( 	const image_geometry::PinholeCameraModel &camera_model,
				const pcl::PointCloud<PointT> in_cloud,
				const cv::Mat in_image,
				pcl::PointCloud<pcl::PointXYZRGB> &out_cloud,
				float min_color_value = 0,
				float min_distance = 1)
{
	for(int i=0; i< in_cloud.size(); ++i){
	   if(  in_cloud.at(i).z > min_distance){ // min distance from camera 1m
			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in_cloud.at(i).x, in_cloud.at(i).y, in_cloud.at(i).z));

			if( ( point_image.x > 0 &&  point_image.x < in_image.cols )
				&& ( point_image.y > 0 &&  point_image.y < in_image.rows )
				)
			{
				// Get image Color
				Image_element_T color = in_image.at<Image_element_T>(point_image);
				if( color.val[0] > min_color_value
					|| color.val[1] > min_color_value
					|| color.val[2] > min_color_value
					)
				{
					// Create a colored point
					pcl::PointXYZRGB color_point(color.val[2], color.val[1], color.val[0]);
					color_point.x = in_cloud.at(i).x;
					color_point.y = in_cloud.at(i).y;
					color_point.z = in_cloud.at(i).z;
					out_cloud.points.push_back(color_point);
				}
			}
	   }
	}
}

template <typename PointT, typename Image_element_T>
inline void
pointcloud_rgb( 	const Projected_pointcloud<PointT> &in_projected,
					const cv::Mat &in_image,
					pcl::PointCloud<pcl::PointXYZRGB> &out_cloud,
					float min_color_value = 0)
{

	for(int i=0; i< in_projected.size(); ++i){
		// Get image Color
		Image_element_T color = in_image.at<Image_element_T>(in_projected.at(i).cv);
		if( color.val[0] > min_color_value
			|| color.val[1] > min_color_value
			|| color.val[2] > min_color_value
			)
		{
			// Create a colored point
			pcl::PointXYZRGB color_point(color.val[2], color.val[1], color.val[0]);
			color_point.x = in_projected.at(i).pcl.x;
			color_point.y = in_projected.at(i).pcl.y;
			color_point.z = in_projected.at(i).pcl.z;
			out_cloud.points.push_back(color_point);
		}
	}
}

}

#endif
