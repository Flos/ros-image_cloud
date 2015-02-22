#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include <eigen3/Eigen/Core>
#include <image_cloud/common/small_helpers.hpp>

#ifndef PCL_DEPTH_FILTER_NEIGHBORS_H_
#define PCL_DEPTH_FILTER_NEIGHBORS_H_

namespace filter_3d
{

template <typename PointT>
inline void
depth_filter_neighbors(
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		int neighbors = 2,
		float epsilon = 0.5,
		float distance_max = 20,
		float threshold = 0.2
)
{
	//std::cout << "neigbors " << neighbors << " epsilon " << epsilon << " distance_max " << distance_max <<std::endl;

	boost::shared_ptr<pcl::search::KdTree<PointT> > tree_n( new pcl::search::KdTree<PointT>() );
	tree_n->setInputCloud(in.makeShared());
	tree_n->setEpsilon(epsilon);

	for(int i = 0; i< in.points.size(); ++i){
		std::vector<int> k_indices;
		std::vector<float> square_distance;

		if ( in.points.at(i).z > distance_max || in.points.at(i).z < 0) continue;

		//Position in image is known
		tree_n->nearestKSearch(in.points.at(i), neighbors, k_indices, square_distance);

		//std::cout << "hier " << i << " z " << in.points.at(i).z  <<std::endl;


		// look for neighbors where at least 1 is closer (Distance to origin) than the other
		int border = 0;
		float min_x = in.points.at(i).x;
		float max_x = min_x;
		float min_y = in.points.at(i).y;
		float max_y = min_y;
		float min_z = in.points.at(i).z;
		float max_z = min_z;

		for(int n = 0; n < k_indices.size(); ++n){
			// is current point an edge point?
			//if(!inRange<float>(in.points.at(k_indices.at(n)).y, 0.005, in.points.at(i).y )) continue;
			if(square_distance.at(n) > 0.01) continue;

			if(in.points.at(k_indices.at((n))).x > max_x)
			{
				max_x = in.points.at(k_indices.at((n))).x;
			}
			if(in.points.at(k_indices.at((n))).x < min_x)
			{
				min_x = in.points.at(k_indices.at((n))).x;
			}
			if(in.points.at(k_indices.at((n))).y > max_y)
			{
				max_y = in.points.at(k_indices.at((n))).y;
			}
			if(in.points.at(k_indices.at((n))).y < min_y)
			{
				min_y = in.points.at(k_indices.at((n))).y;
			}
			if(in.points.at(k_indices.at((n))).z > max_z)
			{
				max_z = in.points.at(k_indices.at((n))).z;
			}
			if(in.points.at(k_indices.at((n))).z < min_z)
			{
				min_z = in.points.at(k_indices.at((n))).z;
			}
		}

		//if( (!inRange(min_y, threshold, in.points.at(i).y)
			//&& !inRange(max_y, threshold, in.points.at(i).y)

			//|| min_y == in.points.at(i).y
			//|| inRange(max_y, threshold, in.points.at(i).y)
			//)
				//&& inRange(max_z, threshold, in.points.at(i).z)
				//&& inRange(min_z, threshold, in.points.at(i).z)
		//)
		if(max_z - threshold > in.points.at(i).z)
			{
//		if( min_x + delta <=  in.points.at(i).x
//			|| max_x == in.points.at(i).x
////			|| min_y == in.points.at(i).y
////			|| max_y == in.points.at(i).y
//			|| min_z == in.points.at(i).z
//			|| max_z == in.points.at(i).z )
//		{
			out.push_back(in.points.at(i));
		}
	}
}

}
#endif
