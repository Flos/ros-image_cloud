#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/structs.hpp>
#include <image_cloud/common/calibration/score.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_neighbors.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <math.h>
#include <opencv2/core/core.hpp>

#ifndef MUTLI_SCORE_H_
#define MUTLI_SCORE_H_

namespace score
{

template <typename PointT, typename ImageT>
inline void
multi_score(
		std::vector<Projected_pointcloud<PointT> > &idx,
		std::vector<cv::Mat> &edge_images,
		long unsigned &score)
{
	assert(idx.size() == edge_images.size());

	for(int i = 0; i< idx.size(); ++i){
		long unsigned score_temp;
		objective_function(idx.at(i), edge_images.at(i), score_temp);
		score += score_temp;
	}
}

template <typename PointT, typename ImageT>
inline void
multi_score(
		std::deque<Projected_pointcloud<PointT> > &idx,
		std::deque<cv::Mat> &edge_images,
		long unsigned &score)
{
	assert(idx.size() == edge_images.size());

	for(int i = 0; i< idx.size(); ++i){
		long unsigned score_temp;
		objective_function(idx.at(i), edge_images.at(i), score_temp);
		score += score_temp;
	}
}


/****
 * Transforms Pointclouds and calculates a result using the objective_function
 */
template <typename PointT, typename ImageT>
inline void
multi_score(
		const image_geometry::PinholeCameraModel &camera_model,
		const std::vector<pcl::PointCloud<PointT> >& pointclouds,
		const std::vector<cv::Mat> &edge_images,
		search::Search_value &search
		)
{
	assert(pointclouds.size() == edge_images.size());


	for(int i = 0; i< pointclouds.size(); ++i){
		long unsigned score_temp;
		pcl::PointCloud<PointT> transformed;

		image_cloud::transform_pointcloud<PointT>(pointclouds.at(i), transformed, search.x, search.y, search.z, search.roll, search.pitch, search.yaw);

		objective_function<PointT, ImageT>( camera_model, transformed, edge_images.at(i), score_temp);

		search.score += score_temp;
	}
}

/****
 * Transforms Pointclouds and calculates a result using the objective_function
 */
template <typename PointT, typename ImageT>
inline void
multi_score(
		const image_geometry::PinholeCameraModel &camera_model,
		const std::deque<pcl::PointCloud<PointT> >& pointclouds,
		const std::deque<cv::Mat> &edge_images,
		search::Search_value &search
		)
{
	assert(pointclouds.size() == edge_images.size());


	for(int i = 0; i< pointclouds.size(); ++i){
		long unsigned score_temp;
		pcl::PointCloud<PointT> transformed;

		image_cloud::transform_pointcloud<PointT>(pointclouds.at(i), transformed, search.x, search.y, search.z, search.roll, search.pitch, search.yaw);

		objective_function<PointT, ImageT>( camera_model, transformed, edge_images.at(i), score_temp);

		search.score += score_temp;
	}
}



/****
 * Transforms Pointclouds and calculates a result using the objective_function
 */
template <typename PointT, typename ImageT>
inline void
multi_score_filter_depth(
		const image_geometry::PinholeCameraModel &camera_model,
		const std::vector<pcl::PointCloud<PointT> >& pointclouds,
		const std::vector<cv::Mat> &edge_images,
		search::Search_value &search
		)
{
	assert(pointclouds.size() == edge_images.size());


	for(int i = 0; i< pointclouds.size(); ++i){
		long unsigned score_temp;
		pcl::PointCloud<PointT> transformed, filtered;

		image_cloud::transform_pointcloud<PointT>(pointclouds.at(i), transformed, search.x, search.y, search.z, search.roll, search.pitch, search.yaw);

		filter_3d::filter_depth_intensity( camera_model, transformed, filtered, edge_images.at(i).rows, edge_images.at(i).cols);

		objective_function<PointT, ImageT>( camera_model, filtered, edge_images.at(i), score_temp);

		search.score += score_temp;
	}
}


/****
 * Transforms Pointclouds and calculates a result using the objective_function
 */
template <typename PointT, typename ImageT>
inline void
multi_score_filter_depth(
		const image_geometry::PinholeCameraModel &camera_model,
		const std::deque<pcl::PointCloud<PointT> >& pointclouds,
		const std::deque<cv::Mat> &edge_images,
		search::Search_value &search
		)
{
	assert(pointclouds.size() == edge_images.size());


	for(int i = 0; i< pointclouds.size(); ++i){
		long unsigned score_temp;
		pcl::PointCloud<PointT> transformed, filtered;

		image_cloud::transform_pointcloud<PointT>(pointclouds.at(i), transformed, search.x, search.y, search.z, search.roll, search.pitch, search.yaw);

		filter_3d::filter_depth_intensity<PointT>( camera_model, transformed, filtered, edge_images.at(i).rows, edge_images.at(i).cols);

		objective_function<PointT, ImageT>( camera_model, filtered, edge_images.at(i), score_temp);

		search.score += score_temp;
	}
}

}

#endif
