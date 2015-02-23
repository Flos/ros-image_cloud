#ifndef EDGE_IMAGE_PLANE_H_
#define EDGE_IMAGE_PLANE_H_

#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/filter/pcl/common.hpp>
#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>


namespace filter_3d{

template <typename PointT, typename ImageT>
void
edge_image_plane(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		int rows,
		int cols,
		float threashold = 1,
		project2d::Field field = project2d::DEPTH
		)
{
		project2d::Points2d<PointT> point_map;
		cv::Mat depth_map = cv::Mat::zeros(rows, cols, cv::DataType<ImageT>::type);

		point_map.init(camera_model, in, depth_map, field);

		// Apply edge filter
		//imwrite("depth_map_1_DEPTH.jpg",depth_map);
		cv::blur(depth_map, depth_map, cv::Size( 3, 3 ), cv::Point(-1,-1));
		//imwrite("depth_map_blur.jpg",depth_map);
		cv::Canny(depth_map, depth_map, 100, 200, 5);
		//imwrite("depth_map_2_canny.jpg",depth_map);

		point_map.template get_points<ImageT>(depth_map, out, threashold);


}

}

#endif
