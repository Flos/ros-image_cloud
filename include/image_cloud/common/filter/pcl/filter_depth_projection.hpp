#ifndef FILTER_DEPTH_PROJECTION_H_
#define FILTER_DEPTH_PROJECTION_H_

#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/project2d.hpp>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

//#define DEBUG
//#define USE_Y_THRESHOLD
//#define USE_SQUERE_DISTANCE

namespace filter_3d{
	template <typename PointT>
	inline float
	distance_to_origin(const PointT &point){
		return sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
	}

	template <typename PointT>
	inline PointT
	delta(const PointT &point1, const PointT &point2){
		PointT p;
		p.x = point2.x - point1.x;
		p.y = point2.y - point1.y;
		p.z = point2.z - point1.z;
		p.intensity = point2.intensity - point1.intensity;

		return p;
	}

	template <typename PointT>
	inline bool
	is_depth_step(const PointT &point1, const PointT &point2){
		float threshold = 0.5;
		float distance_x = (point1.x-point2.x) * (point1.x-point2.x);
		float distance_y = (point1.y-point2.y) * (point1.y-point2.y);
		float sqr_distance_xy = sqrt(distance_x + distance_y);

		float sqr_distance_z = sqrt((point1.z-point2.z) * (point1.z-point2.z));

		// Step into z direction is bigger than in x or y
		//sqr_distance_xy < 3 &&
		return sqr_distance_z - threshold > sqr_distance_xy;
	}

	template <typename PointT>
	inline bool
	is_edge(const pcl::PointCloud<PointT> &in, int current_idx, std::vector<int> &k_indices, std::vector<float> &square_distance){

#ifdef USE_SQUERE_DISTANCE
			const float point_distance = distance_to_origin(in.points.at(current_idx));
#else
			const float point_distance = in.points.at(current_idx).z;
#endif

			float min_z = point_distance;
			float max_z = point_distance;
			float max_y = in.points.at(current_idx).y;
			int best_candidate = current_idx;

			const float z_threshold = 0.3;

#ifdef USE_Y_THRESHOLD
			const float y_threshold = 0.10;

			const float y_min_t = in.points.at(current_idx).y - y_threshold;
			const float y_max_t = in.points.at(current_idx).y + y_threshold;
#endif

			for(int n = 0; n < k_indices.size(); ++n){

#ifdef USE_Y_THRESHOLD
				if(! between( y_min_t, in.points.at(k_indices.at(n)).y, y_max_t)) continue;
#endif

#ifdef USE_SQUERE_DISTANCE
				float distance_n = distance_to_origin(in.points.at(k_indices.at(n)));
#else
				float distance_n = in.points.at(k_indices.at((n))).z;
#endif

				if(distance_n > point_distance){
					max_z = distance_n;
					best_candidate = k_indices.at(n);
				}

//				if(in.points.at(k_indices.at((n))).x > max_x)
//				{
//					max_x = in.points.at(k_indices.at((n))).x;
//				}
//				if(in.points.at(k_indices.at((n))).x < min_x)
//				{
//					min_x = in.points.at(k_indices.at((n))).x;
//				}
//				if(in.points.at(k_indices.at(n)).y > max_y)
//				{
//					max_y = in.points.at(k_indices.at(n)).y;
//				}
//				if(in.points.at(k_indices.at(n)).y < min_y)
//				{
//					min_y = in.points.at(k_indices.at((n))).y;
//				}
//				if(in.points.at(k_indices.at((n))).z < min_z)
//				{
//					min_z = in.points.at(k_indices.at((n))).z;
//				}
			}

			//if( max_z - threshold > in.points.at(current_idx).z)

			if((is_depth_step(in.points.at(current_idx), in.points.at(best_candidate)))
					//|| max_y == in.points.at(current_idx).y /* no neighbor above */
					)
			{
#ifdef DEBUG
				std::cout << "threshold: " <<  z_threshold << " max_z: " << max_z << " point: " <<  in.points.at(current_idx) << " best: " <<  in.points.at(best_candidate) << " delta: " << delta(in.points.at(current_idx), in.points.at(best_candidate)) << std::endl;
#endif
				return true;
			}
#ifdef DEBUG
			//std::cout << "from " << k_indices.size() << " neigbors non matchs the condition"<< std::endl;
#endif
			return false;
	}

	inline void look_up_indices(const std::vector<int>& points2d_indices, std::vector<int>& k_indices) {
		for (int k = 0; k < k_indices.size(); ++k) {
			k_indices[k] = points2d_indices.at(k_indices.at(k));
		}
	}

	template <typename PointT>
	inline void
	filter_depth_projection(	const image_geometry::PinholeCameraModel &camera_model,
								const pcl::PointCloud<PointT> &in,
								pcl::PointCloud<PointT> &out,
								int rows,
								int cols,
								int k_neighbors = 8)
	{
		std::vector<std::vector<bool> > hit( cols, std::vector<bool>(rows));
		std::vector<int> points2d_indices;
		pcl::PointCloud<pcl::PointXY> points2d;

#ifdef DEBUG
		std::cout << "in points: "<< in.size() << "\n";
#endif

		// Project points into image space
		for(unsigned int i=0; i < in.points.size();  ++i){
			const PointT* pt = &in.points.at(i);
			if( pt->z > 1) { // min distance from camera 1m

				cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(pt->x, pt->y, pt->z));

				if( between<int>(0, point_image.x, cols )
					&& between<int>( 0, point_image.y, rows )
				)
				{
					// Point allready at this position?
					if(!hit[point_image.x][point_image.y]){
						hit[point_image.x][point_image.y] = true;

						pcl::PointXY p_image;
						p_image.x = point_image.x;
						p_image.y = point_image.y;

						points2d.push_back(p_image);
						points2d_indices.push_back(i);
					}

				}
			}
		}

#ifdef DEBUG
		std::cout << "projected 2d points: "<< points2d.size() << " indices: " << points2d_indices.size() << "\n";
#endif

		pcl::search::KdTree<pcl::PointXY> tree_n;
		tree_n.setInputCloud(points2d.makeShared());
		tree_n.setEpsilon(0.1);

		for(unsigned int i=0; i < points2d.size(); ++i){
			std::vector<int> k_indices;
			std::vector<float> square_distance;

			//tree_n.nearestKSearch(points2d.at(i), k_neighbors, k_indices, square_distance);
			tree_n.radiusSearch(points2d.at(i), k_neighbors, k_indices, square_distance);

			look_up_indices(points2d_indices, k_indices);

			if(is_edge(in, points2d_indices.at(i), k_indices, square_distance)){
				out.push_back(in.points.at(points2d_indices.at(i)));
			}
		}

#ifdef DEBUG
		std::cout << "out 2d points: "<< out.size() << "\n";
#endif
	}
}
#endif
