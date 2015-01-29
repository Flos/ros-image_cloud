#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/structs.hpp>
#include <image_cloud/common/calibration/score.hpp>

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
		objective_function(idx.at(i), edge_images.at(i),score_temp);
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

		Projected_pointcloud<PointT> p;

		project2d::project_2d<PointT>( camera_model, transformed, p, edge_images.at(i).cols, edge_images.at(i).rows );
		objective_function<PointT, ImageT>( p, edge_images.at(i), score_temp);

		search.result += score_temp;
	}
}


}

#endif
