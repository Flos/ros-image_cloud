#ifndef IMAGE_CLOUD_COMMON_PIPELINE_POINTCLOUD_H_
#define IMAGE_CLOUD_COMMON_PIPELINE_POINTCLOUD_H_

#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/filter/pcl/depth_edge.hpp>
#include <image_cloud/common/filter/pcl/normal_diff_filter.hpp>
#include <image_cloud/common/filter/pcl/range_borders.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_radius.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_neighbors.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_projection.hpp>
#include <image_cloud/common/filter/pcl/hit_same_point.hpp>
#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/calibration/pipeline/enums.h>
#include <assert.h>
#include <opencv2/opencv.hpp>

namespace image_cloud{

template <typename PointT>
inline void
filter3d_switch(const pcl::PointCloud<PointT> &pcl_in,
		pcl::PointCloud<PointT> &pcl_out,
		image_geometry::PinholeCameraModel &camera_model,
		int camera,
		int sequence,
		pcl_filter::Filter3d filter,
		int rows=0,
		int cols=0){

	switch (filter)
	{
		case pcl_filter::OFF:
				pcl_out = pcl_in;
			break;
		case pcl_filter::DEPTH:
		{
			filter_3d::filter_depth_discontinuity(pcl_in, pcl_out); // epsilon
		}
		break;

		case pcl_filter::DEPTH_INTENSITY:
		{
			std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(cols,
									std::vector<boost::shared_ptr<pcl::PointXYZI> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, pcl_in, map, projected_pointclouds, cols, rows);
			filter_3d::filter_depth_intensity(map, pcl_out);
		}
		break;

		case pcl_filter::DEPTH_EDGE:
		{
			std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(cols,
												std::vector<boost::shared_ptr<pcl::PointXYZI> > (rows));
			Projected_pointcloud<PointT> projected_pointclouds;
			project2d::project_2d(camera_model, pcl_in, map, projected_pointclouds, cols, rows);
			filter_3d::depth_edge(map, pcl_out); // neighbors
		}
		break;
		case pcl_filter::NORMAL_DIFF:
		{
			filter_3d::normal_diff_filter(pcl_in, pcl_out); // threshold
		}
		break;
		case pcl_filter::RANGE_BORDERS:
		{
			filter_3d::range_borders(pcl_in, pcl_out); // threshold
		}
		break;
		case pcl_filter::DEPTH_RADIUS:
		{
			filter_3d::depth_discontinuity_radius(pcl_in, pcl_out); // min neighbors
		}
		break;
		case pcl_filter::DEPTH_NEIGHBORS:
		{
			filter_3d::depth_filter_neighbors(pcl_in, pcl_out); // max distance
		}
		break;
		case pcl_filter::DEPTH_EDGE_PROJECTION:
		{
			filter_3d::filter_depth_projection(camera_model, pcl_in, pcl_out, rows, cols); // neighbors); // max distance
		}
		break;
		case pcl_filter::HIT_SAME_POINT:
		{
			filter_3d::hit_same_point(camera_model, pcl_in, pcl_out, rows, cols);
		}
		break;
		case pcl_filter::OTHER:
		{
			filter_3d::segmentation(pcl_in, pcl_out);
				//filtred = transformed;
		}
		break;
		default:
			break;
	}
}



}

#endif
