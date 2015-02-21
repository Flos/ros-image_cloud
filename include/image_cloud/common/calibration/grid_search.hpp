#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>
#include <image_cloud/common/calibration/structs.hpp>

#include <deque>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <math.h>

#ifndef SEARCH_GRID_6D_H_
#define SEARCH_GRID_6D_H_

//#define debug
namespace search
{
	inline void grid_setup(Search_setup setup, std::vector<Search_value>& results){
		results.push_back(Search_value(setup.x.center, setup.y.center, setup.z.center, setup.roll.center, setup.pitch.center, setup.yaw.center)); // first value is origin
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
								// Origin is always the first element, dont add it again
								if(setup.x.center == setup.x.at(x)
										&& setup.y.center == setup.y.at(y)
										&& setup.z.center == setup.z.at(z)
										&& setup.roll.center == setup.roll.at(roll)
										&& setup.pitch.center == setup.pitch.at(pitch)
										&& setup.yaw.center == setup.yaw.at(yaw))
								{
#ifdef debug
									std::cout << results.size()-1 << "\t" << x << y << z<< roll<< pitch << yaw << std::endl;
#endif
									continue;
								}

								results.push_back(Search_value(setup.x.at(x), setup.y.at(y), setup.z.at(z), setup.roll.at(roll), setup.pitch.at(pitch), setup.yaw.at(yaw)));
#ifdef debug
								std::cout << results.size()-1 << "\t" << results.at(results.size()-1).to_simple_string() << std::endl;
#endif
							}
						}
					}
				}
			}
		}
#ifdef debug
		std::cout << "grid_setup: " << results.size();
#endif
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

	inline bool sortByscore(const Search_value &lhs, const Search_value &rhs) { return lhs.score > rhs.score; }

	inline void evaluate_results(std::vector<Search_value> &result_list,
			tf::Transform& out,
			Multi_search_result* multi_result = NULL)
	{
		unsigned long int worse = 0;
		unsigned long int best_result = 0;
		unsigned long int best_result_idx = 0;
		for (int i = 1; i < result_list.size(); ++i) {
			if (result_list.at(i).score > best_result) {
				best_result_idx = i;
				best_result = result_list.at(i).score;
			}
			if (result_list.at(0).score > result_list.at(i).score) {
				++worse;
			}
		}
		result_list.at(best_result_idx).get_transform(out);
		if (multi_result != NULL) {
			multi_result->best = result_list.at(best_result_idx);
			multi_result->center = result_list.at(0);
			multi_result->nr_worse = worse;
			multi_result->nr_total = result_list.size();
			multi_result->best_results.assign(result_list.begin(), result_list.end());
			//Sort results by score
			std::sort(multi_result->best_results.begin(), multi_result->best_results.end(), sortByscore);

		}
}



	template <typename PointT, typename ImageT>
	inline void get_best_tf(std::vector<Search_value> result_list,
						tf::Transform &out,
						const image_geometry::PinholeCameraModel &camera_model,
						const std::deque<pcl::PointCloud<PointT> > &pointclouds,
						const std::deque<cv::Mat> &images,
						bool pre_filtred = true,
						Multi_search_result *multi_result = NULL)
	{
		calculate<PointT, ImageT>( camera_model, pointclouds, images, result_list, pre_filtred);

		evaluate_results(result_list, out, multi_result);
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

		get_best_tf<PointT,ImageT>(result_list, out, camera_model, pointclouds, images, pre_filtred, multi_result);
	}

}

#endif
