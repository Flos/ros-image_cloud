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
	inline void grid_setup(Search_setup setup, std::vector<Search_value>& results){
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
	inline void calculate(const image_geometry::PinholeCameraModel &camera_model, const std::vector<pcl::PointCloud<PointT> > &pointclouds, const std::vector<cv::Mat> &edge_images, std::vector<Search_value>& results, bool pre_filtred=true){
		for(int i=0; i < results.size(); ++i){
			if(pre_filtred)
			{
				score::multi_score<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));

			}
			else{
				score::multi_score_filter_depth<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
			}
		}
	}

	template <typename PointT, typename ImageT>
	inline void calculate(const image_geometry::PinholeCameraModel &camera_model, const std::deque<pcl::PointCloud<PointT> > &pointclouds, const std::deque<cv::Mat> &edge_images, std::vector<Search_value>& results, bool pre_filtred=true){
		for(int i=0; i < results.size(); ++i){
			if(pre_filtred)
			{
				score::multi_score<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
			}
			else{
				score::multi_score_filter_depth<PointT, ImageT>(camera_model, pointclouds, edge_images, results.at(i));
			}
		}
	}

	template <typename PointT, typename ImageT>
	inline void get_best_tf(	tf::Transform in,
						tf::Transform &out,
						const image_geometry::PinholeCameraModel &camera_model,
						const std::deque<pcl::PointCloud<PointT> > &pointclouds,
						const std::deque<cv::Mat> &images,
						float range_axis = 0.5,
						float range_rot = 0.5,
						int steps = 3,
						bool pre_filtred = true,
						Multi_search_result *multi_result = NULL)
	{
		Search_setup search_range;
		std::vector<Search_value> result_list;


		double r,p,y;
		in.getBasis().getRPY(r, p, y);
		search_range.x.init_range(in.getOrigin()[0], range_axis, steps);
		search_range.y.init_range(in.getOrigin()[1], range_axis, steps);
		search_range.z.init_range(in.getOrigin()[2], range_axis, steps);
		search_range.roll.init_range(r, range_rot, steps);
		search_range.pitch.init_range(p, range_rot, steps);
		search_range.yaw.init_range(y, range_rot, steps);

		grid_setup(search_range, result_list);

		Search_value origin_empty;
		origin_empty.init(in);

		// Add origin
		if(result_list.size() %2 == 0){
			result_list.insert(result_list.begin()+(result_list.size()/2), origin_empty);
		}

		calculate<PointT, ImageT>( camera_model, pointclouds, images, result_list, pre_filtred);

		// get origin
		Search_value origin = result_list.at( (result_list.size()) / 2);
		std::cout << "center"<< spacer << origin.to_string() << "\n";
		std::cout << "empty" << spacer << origin_empty.to_string() << "\n";
//
		assert(origin.x == origin_empty.x);
		assert(origin.y == origin_empty.y);
		assert(origin.z == origin_empty.z);
		assert(origin.roll == origin_empty.roll);
		assert(origin.pitch == origin_empty.pitch);
		assert(origin.yaw == origin_empty.yaw);
		assert(origin.result != origin_empty.result);

		long unsigned int worse = 0;
		long unsigned int best_result = 0;
		long unsigned int best_result_idx = 0;

		for(int i=0; i< result_list.size(); ++i){
			if(result_list.at(i).result > best_result){
				best_result_idx = i;
				best_result = result_list.at(i).result;
			}
			if( origin.result > result_list.at(i).result ){
				++worse;
			}
		}

		result_list.at(best_result_idx).get_transform(out);

		if(multi_result != NULL){
			multi_result->best = result_list.at(best_result_idx);
			multi_result->in = origin;
			multi_result->nr_worse = worse;
			multi_result->nr_total = result_list.size();
		}
	}

}

#endif
