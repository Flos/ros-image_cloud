#ifndef FILDER_DEPTH_INTENSITY_H_
#define FILDER_DEPTH_INTENSITY_H_

#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/project2d.hpp>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>



namespace filter_3d{

/***
 * Returns Point3i
 * x = hit intensity out of range
 * z = hit depth out of range
 */
	template<typename PointT>
	cv::Point3i inline
	check(PointT &point,
				 float& min_intensity,
				 float& max_intensity,
				 float& min_depth,
				 float& max_depth)
	{
		cv::Point3i hits;
		if (!between(min_intensity, point.intensity, max_intensity)) {
			++hits.x;
		}
		if (!between(min_depth, point.z, max_depth)) {
			++hits.z;
		}
		return hits;
	}

	template <typename PointT>
	inline cv::Point3i
	search(int x, int y,
					int steps_left,
					std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
					float &min_intensity,
					float &max_intensity,
					float &min_depth,
					float &max_depth,
					bool &direction_x )
	{
		if( idx[x][y] )
		{
			return check(*idx[x][y], min_intensity, max_intensity, min_depth, max_depth);
		}
		else{
			if(direction_x && (x+1) < idx.size()){
				return search(x+1, y, steps_left -1, idx, min_intensity, max_intensity, min_depth, max_depth, direction_x);
			}
			else if(!direction_x && (y+1) < idx[0].size()){
				return search(x, y+1, steps_left -1, idx, min_intensity, max_intensity, min_depth, max_depth, direction_x);
			}
		}
		return cv::Point3i(0,0,0);
	}


	template <typename PointT>
	inline cv::Point3i
	start(int x, int y,
					int steps_left,
					std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
					float &range_intensity,
					float &range_depth,
					bool &direction_x
				)
	{
		float intensity_min = idx[x][y]->intensity - range_intensity;
		float intensity_max = idx[x][y]->intensity + range_intensity;
		float depth_min = idx[x][y]->z - range_depth;
		float depth_max = idx[x][y]->z + range_depth;


		if(direction_x && x+1< idx.size()){
			return search(x+1, y, steps_left -1, idx, intensity_min, intensity_max, depth_min, depth_max, direction_x);
		}
		else if(!direction_x && y+1 < idx[0].size()){
			return search(x, y+1, steps_left -1, idx, intensity_min, intensity_max, depth_min, depth_max, direction_x);
		}
		else{
			return cv::Point3i(0,0,0);
		}
	}


	template <typename PointT>
	inline void
	filter_depth_intensity(
				std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
				pcl::PointCloud<PointT> &out,
				float range_depth = 1.4,
				float range_intensity = 200,
				int range_search = 3,
				bool direction_x = true
		)
	{
		assert( out.empty());

		unsigned int hits = 0;

		//iterate over all pixel and filter edges;
		for(int y = 0; y < idx[0].size(); y++)
			{
				for(int x = 0; x < idx.size(); x++)
				{

				cv::Point3i value;
				if( idx[x][y]){ /* found something */

					value = start( x , y, range_search, idx, range_intensity, range_depth, direction_x);

					if( value.x != 0 && range_intensity != 0){
						out.push_back(*idx[x][y]);
					}
					else if( value.z != 0 && range_depth != 0){

						out.push_back(*idx[x][y]);
					}
				}
			}
		}
	}

	template <typename PointT>
	inline void
	filter_depth_intensity(	const image_geometry::PinholeCameraModel &camera_model,
								const pcl::PointCloud<PointT> &in_points,
								pcl::PointCloud<PointT> &out_points,
								int rows,
								int cols,
								float range_depth = 0.3,
								float range_intensity = 50)
	{


		std::vector<std::vector<boost::shared_ptr<PointT> > > pointcloud_map( cols, std::vector<boost::shared_ptr<PointT> >(rows));

		project2d::project_2d<PointT>(camera_model, in_points, pointcloud_map, true);

		filter_3d::filter_depth_intensity<PointT>(pointcloud_map, out_points, range_depth, range_intensity);
	}
}
#endif
