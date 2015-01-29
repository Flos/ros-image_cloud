#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>
#include <image_cloud/common/calibration/structs.hpp>

#include <opencv2/core/core.hpp>
#include <math.h>

#ifndef SEARCH_GRID_6D_H_
#define SEARCH_GRID_6D_H_

namespace search
{
	void grid_setup(Search_setup setup, std::vector<Search_value>& results){
		for(int x=0; x < setup.x.steps_max; ++x)
		{
			for(int y=0; y < setup.y.steps_max; ++y)
			{
				for(int z=0; z < setup.z.steps_max; ++z)
				{
					for(int roll=0; roll < setup.roll.steps_max; ++roll)
					{
						for(int pitch=0; pitch < setup.pitch.steps_max; ++pitch)
						{
							for(int yaw=0; yaw < setup.yaw.steps_max; ++yaw)
							{
								results.push_back(Search_value(setup.x.at(x), setup.y.at(y), setup.z.at(z), setup.roll.at(roll), setup.pitch.at(pitch), setup.yaw.at(yaw)));
							}
						}
					}
				}
			}
		}
	}

	template <typename PointT, typename ImageT>
	inline void calculate(const image_geometry::PinholeCameraModel &camera_model, const std::vector<pcl::PointCloud<PointT> > &pointclouds, const std::vector<cv::Mat> &edge_images, std::vector<Search_value>& results){
		for(int i=0; i < results.size(); ++i){
			score::multi_score<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
		}
	}
}

#endif
