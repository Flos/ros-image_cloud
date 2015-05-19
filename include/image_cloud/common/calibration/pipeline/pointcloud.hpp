#ifndef IMAGE_CLOUD_COMMON_PIPELINE_POINTCLOUD_H_
#define IMAGE_CLOUD_COMMON_PIPELINE_POINTCLOUD_H_

#include <image_cloud/common/filter/pcl/common.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/filter/pcl/depth_edge.hpp>
#include <image_cloud/common/filter/pcl/normal_diff_filter.hpp>
#include <image_cloud/common/filter/pcl/range_borders.hpp>
#include <image_cloud/common/filter/pcl/remove_cluster_2d.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_radius.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_neighbors.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_discontinuity.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_projection.hpp>
#include <image_cloud/common/filter/pcl/hit_same_point.hpp>
#include <image_cloud/common/filter/pcl/edge_image_plane.hpp>
#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/calibration/pipeline/enums.h>
#include <assert.h>
#include <opencv2/opencv.hpp>

namespace image_cloud{

template <typename PointT>
inline void
filter3d_switch(const pcl::PointCloud<PointT> &in_points,
		pcl::PointCloud<PointT> &out_points,
		const image_geometry::PinholeCameraModel &camera_model,
		pcl_filter::Filter3d filter,
		int rows=0,
		int cols=0){

	switch (filter)
	{
		case pcl_filter::OFF:
				out_points = in_points;
			break;
		case pcl_filter::DEPTH:
		{
			filter_3d::filter_depth_discontinuity(in_points, out_points); // epsilon
		}
		break;

		case pcl_filter::DEPTH_INTENSITY:
		{
			std::vector<std::vector<boost::shared_ptr<PointT> > > map(cols,
									std::vector<boost::shared_ptr<PointT> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, in_points, map, projected_pointclouds, cols, rows);
			filter_3d::filter_depth_intensity<PointT>(map, out_points);
		}
		break;
		case pcl_filter::DEPTH_EDGE:
		{
			std::vector<std::vector<boost::shared_ptr<PointT> > > map(cols,
												std::vector<boost::shared_ptr<PointT> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, in_points, map, projected_pointclouds, cols, rows);
			filter_3d::depth_edge<PointT>(map, out_points); // neighbors
		}
		break;
		case pcl_filter::NORMAL_DIFF:
		{
			filter_3d::normal_diff_filter<PointT>(in_points, out_points); // threshold
		}
		break;
		case pcl_filter::REMOVE_CLUSTER_2D:
		{
			filter_3d::remove_cluster_2d<PointT>(camera_model, in_points, out_points, rows, cols); // threshold
		}
		break;
		case pcl_filter::RANGE_BORDERS:
		{
			filter_3d::range_borders<PointT>(in_points, out_points); // threshold
		}
		break;
		case pcl_filter::DEPTH_RADIUS:
		{
			filter_3d::depth_discontinuity_radius<PointT>(in_points, out_points); // min neighbors
		}
		break;
		case pcl_filter::DEPTH_NEIGHBORS:
		{
			filter_3d::depth_filter_neighbors<PointT>(in_points, out_points); // max distance
		}
		break;
		case pcl_filter::DEPTH_EDGE_PROJECTION:
		{
			filter_3d::filter_depth_projection<PointT>(camera_model, in_points, out_points, rows, cols); // neighbors); // max distance
		}
		break;
		case pcl_filter::DEPTH_NEIGHBOR_DISONTINUITY:
		{
			filter_3d::filter_depth_discontinuity<PointT>(in_points, out_points, 2, 0.3, 1, 300); // neighbors, min, max, epsilon); // max distance
		}
		break;
		case pcl_filter::HIT_SAME_POINT:
		{
			filter_3d::hit_same_point<PointT>(camera_model, in_points, out_points, rows, cols);
		}
		break;
		case pcl_filter::OTHER:
		{
			filter_3d::segmentation<PointT>(in_points, out_points);
				//filtred = transformed;
		}
		break;
		case pcl_filter::DEPTH_INTENSITY_NORMAL_DIFF:
		{
			std::vector<std::vector<boost::shared_ptr<PointT> > > map(cols,
									std::vector<boost::shared_ptr<PointT> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, in_points, map, projected_pointclouds, cols, rows);

			pcl::PointCloud<PointT> temp_points;
			filter_3d::filter_depth_intensity<PointT>(map, temp_points);
			filter_3d::depth_discontinuity_radius<PointT>(temp_points, out_points, 0.5, 0.05, 70); // min neighbors

			//filter_3d::normal_diff_filter<PointT>(temp_points, out_points);
		}
		break;
		case pcl_filter::DEPTH_INTENSITY_AND_REMOVE_CLUSER_2D:
		{
			std::vector<std::vector<boost::shared_ptr<PointT> > > map(cols,
			std::vector<boost::shared_ptr<PointT> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, in_points, map, projected_pointclouds, cols, rows);

			pcl::PointCloud<PointT> temp_points;
			filter_3d::filter_depth_intensity<PointT>(map, temp_points);
			filter_3d::remove_cluster_2d<PointT>(camera_model, temp_points, out_points, rows, cols); // threshold
		}
		break;
		case pcl_filter::REMOVE_CLUSER_2D_RADIUS_SEARCH:
		{
			pcl::PointCloud<PointT> temp_points;
			filter_3d::remove_cluster_2d<PointT>(camera_model, in_points, temp_points, rows, cols); // threshold
			filter_3d::depth_discontinuity_radius<PointT>(temp_points, out_points); // min neighbors
		}
		break;
		case pcl_filter::EDGE_IMAGE_PLANE:
		{
			filter_3d::edge_image_plane<PointT,uchar>(camera_model, in_points, out_points, rows, cols);
		}
		break;
		case pcl_filter::EDGE_IMAGE_PLANE_2D_RADIUS_SEARCH:
		{
			pcl::PointCloud<PointT> temp_points;
			filter_3d::edge_image_plane<PointT,uchar>(camera_model, in_points, temp_points, rows, cols);
			filter_3d::remove_cluster_2d<PointT>(camera_model,temp_points, out_points, rows, cols); // min neighbors
		}
		break;
		case pcl_filter::EDGE_IMAGE_PLANE_NORMAL_DIFF:
		{
			pcl::PointCloud<PointT> temp_points;
			filter_3d::edge_image_plane<PointT,uchar>(camera_model, in_points, temp_points, rows, cols);
			filter_3d::normal_diff_filter<PointT>(temp_points, out_points, 0.001,0.1);
		}
		break;
		case pcl_filter::DEPTH_EDGE_PROJECTION_AGGREGATED:
		{
			filter_3d::filter_depth_projection<PointT>(camera_model, in_points, out_points, rows, cols, 2, 1.5); // neighbors); // max distance
		}
		break;
		default:
			break;
	}
}



}

#endif
