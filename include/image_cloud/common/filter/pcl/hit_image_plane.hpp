#ifndef FILDER_HIT_IMAGE_PLANE_H_
#define FILDER_HIT_IMAGE_PLANE_H_

#include <image_cloud/common/small_helpers.hpp>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>


namespace filter{

template <typename PointT>
void
hit_image_plane(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		int image_width,
		int image_height)
{
		BOOST_FOREACH (PointT &pt, in.points){
			if( pt.z > 1) { // min distance from camera 1m

				cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

				if( between<int>(0, point_image.x, image_width )
					&& between<int>( 0, point_image.y, image_height )
				)
				{
					// Point in image push to 2d and 3d point
					out.push_back(pt);
				}
			}
		}
}

}
