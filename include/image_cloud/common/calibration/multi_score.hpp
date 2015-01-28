#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <common/type.hpp>
#include <math.h>
#include <common/calibration/structs.hpp>
#include <common/transform.hpp>

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
		std::vector<pcl::PointCloud<PointT> > pointclouds,
		std::vector<cv::Mat> &edge_images,
		search::Search_value &search
		)
{
	assert(pointclouds.size() == edge_images.size());

	for(int i = 0; i< pointclouds.size(); ++i){
		long unsigned score_temp;

		image_cloud::transform_pointcloud(pointclouds.at(i), search.x, search.y, search.z, search.roll, search.pitch, search.yaw);

		Projected_pointcloud<PointT> p;
		p.image_size.heigh = edge_images.at(i).rows;
		p.image_size.width = edge_images.at(i).cols;

		project2d::project_2d( camera_model, pointclouds.at(i), p);

		objective_function( pointclouds.at(i).at(i), edge_images.at(i), score_temp);

		search.result += score_temp;
	}
}


}

#endif
