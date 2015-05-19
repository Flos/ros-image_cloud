#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include <eigen3/Eigen/Core>
#include <image_cloud/common/small_helpers.hpp>

#ifndef PCL_FILTER_DEPTH_DISCONTINUITY_H_
#define PCL_FILTER_DEPTH_DISCONTINUITY_H_
#define USE_SQUERE_DISTANCE

namespace filter_3d
{

template <typename PointT>
inline void
filter_depth_discontinuity(
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		int neighbors = 2,
		float threshold = 0.3,
		float distance_min = 1,
		float distance_max = 300,
		float epsilon = 0.5
)
{
	//std::cout << "neigbors " << neighbors << " epsilon " << epsilon << " distance_max " << distance_max <<std::endl;

	boost::shared_ptr<pcl::search::KdTree<PointT> > tree_n( new pcl::search::KdTree<PointT>() );
	tree_n->setInputCloud(in.makeShared());
	tree_n->setEpsilon(epsilon);

	for(int i = 0; i< in.points.size(); ++i){
		std::vector<int> k_indices;
		std::vector<float> square_distance;

		if ( in.points.at(i).z > distance_max || in.points.at(i).z < distance_min) continue;

		//Position in image is known
		tree_n->nearestKSearch(in.points.at(i), neighbors, k_indices, square_distance);

		//std::cout << "hier " << i << " z " << in.points.at(i).z  <<std::endl;

#ifdef USE_SQUERE_DISTANCE
		const float point_distance = distance_to_origin<PointT>(in.points.at(i));
#else
		const float point_distance = in.points.at(i).z;
#endif
		float value = 0; //point_distance;
		unsigned int idx = 0;
		for(int n = 0; n < k_indices.size(); ++n){

#ifdef USE_SQUERE_DISTANCE
			float distance_n = distance_to_origin<PointT>(in.points.at(k_indices.at(n)));
#else
			float distance_n = in.points.at(k_indices.at((n))).z;
#endif
			if(value < distance_n - point_distance){
				idx = k_indices.at(n);
				value = distance_n - point_distance;
			}
		}

		if(value > threshold){
			out.push_back(in.points.at(i));
			out.at(out.size()-1).intensity = sqrt(value);
		}
	}
}

}
#endif
