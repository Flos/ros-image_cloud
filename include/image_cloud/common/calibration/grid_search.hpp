#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>
#include <image_cloud/common/calibration/structs.hpp>

#include <deque>

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
	void get_best_tf(	tf::Transform in,
						tf::Transform &out,
						const image_geometry::PinholeCameraModel &camera_model,
						const std::deque<pcl::PointCloud<PointT> > &pointclouds,
						const std::deque<cv::Mat> &images,
						float range = 0.5,
						int steps = 3)
	{
		Search_setup search_range;
		std::vector<Search_value> results;

		double r,p,y;
		in.getBasis().getRPY(r, p, y, 1);
		search_range.x.init_range(in.getOrigin()[0], range, steps);
		search_range.y.init_range(in.getOrigin()[1], range, steps);
		search_range.z.init_range(in.getOrigin()[2], range, steps);
		search_range.roll.init_range(r, range, steps);
		search_range.pitch.init_range(p, range, steps);
		search_range.yaw.init_range(y, range, steps);

		grid_setup(search_range, results);

		calculate( camera_model, pointclouds, images, results );

		int best_result_idx = 0;
		long unsigned int best_result = 0;
		for(int i=0; i< results.size(); ++i){
			if(results.at(i).result > best_result){
				best_result_idx = i;
				best_result = results.at(i).result;
			}
		}
		//printf("%d: \t%s\n", best_result_idx, results.at(best_result_idx).to_string().c_str());

		out.setOrigin( tf::Vector3(results.at(best_result_idx).x, results.at(best_result_idx).y, results.at(best_result_idx).z ) );

		tf::Quaternion q;
		q.setRPY(results.at(best_result_idx).roll, results.at(best_result_idx).pitch, results.at(best_result_idx).yaw );
		out.setRotation( q);
	}


	template <typename PointT, typename ImageT>
	inline void calculate(const image_geometry::PinholeCameraModel &camera_model, const std::vector<pcl::PointCloud<PointT> > &pointclouds, const std::vector<cv::Mat> &edge_images, std::vector<Search_value>& results){
		for(int i=0; i < results.size(); ++i){
			score::multi_score<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
		}
	}
}

#endif
