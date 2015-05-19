#ifndef IMAGE_CLOUD_FILTER_PCL_COMMON_HPP_
#define IMAGE_CLOUD_FILTER_PCL_COMMON_HPP_

//#define DEBUG
//#define USE_Y_THRESHOLD
#define USE_SQUERE_DISTANCE

namespace filter_3d{

	inline void look_up_indices(const std::vector<int>& points2d_indices, std::vector<int>& k_indices) {
	for (int k = 0; k < k_indices.size(); ++k) {
		k_indices[k] = points2d_indices.at(k_indices.at(k));
	}
}

template <typename PointT>
inline bool
	is_depth_step(const PointT &point1, const PointT &point2, float threshold = 2){
	float distance_x = (point1.x-point2.x) * (point1.x-point2.x);
	float distance_y = (point1.y-point2.y) * (point1.y-point2.y);
	float sqr_distance_xy = sqrt( distance_x + distance_y);

	float sqr_distance_z = sqrt((point1.z-point2.z) * (point1.z-point2.z));

	// Step into z direction is bigger than in x or y
	//sqr_distance_xy < 3 &&
	return sqr_distance_z > sqr_distance_xy * threshold;
}

	template <typename PointT>
	inline bool
	is_edge_z(const pcl::PointCloud<PointT> &in, int current_idx, std::vector<int> &k_indices, std::vector<float> &square_distance, float &edginess, float threshold = 0.1){

		const float point_distance = in.points.at(current_idx).z;
		edginess = 0;

		for(int n = 0; n < k_indices.size(); ++n){

			float distance_n = in.points.at(k_indices.at((n))).z;

			if(distance_n < point_distance - threshold){
				return false;
			}
			else if(edginess < distance_n - point_distance){
				edginess = distance_n - point_distance;
			}
		}
		return true;
	}

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
	is_edge(const pcl::PointCloud<PointT> &in, int current_idx, std::vector<int> &k_indices, std::vector<float> &square_distance){

#ifdef USE_SQUERE_DISTANCE
			const float point_distance = distance_to_origin(in.points.at(current_idx));
#else
			const float point_distance = in.points.at(current_idx).z;
#endif

			float min_z = point_distance;
			float max_z = 0; //point_distance;
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

//				if(distance_n > point_distance){
//					max_z = distance_n;
//					best_candidate = k_indices.at(n);
//				}

				max_z = MAX(max_z, distance_n -point_distance);

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
			if(max_z > z_threshold) return true;
			return false;

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


	template <typename PointT>
		inline bool
		is_edge_threshold(const pcl::PointCloud<PointT> &in, int current_idx, std::vector<int> &k_indices, std::vector<float> &square_distance, float &value, float threshold = 0.3){

	#ifdef USE_SQUERE_DISTANCE
			const float point_distance = distance_to_origin<PointT>(in.points.at(current_idx));
	#else
			const float point_distance = in.points.at(current_idx).z;
	#endif
			value = 0; //point_distance;
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
			if(value > threshold) return is_depth_step(in.points.at(current_idx), in.points.at(idx), 1.3);
			return false;
	}

	template <typename PointT>
			inline bool
			is_light_edge_threshold(const pcl::PointCloud<PointT> &in, int current_idx, std::vector<int> &k_indices, std::vector<float> &square_distance, float &value, float threshold_min = 0.3, float threshold_max = 0.3){

		#ifdef USE_SQUERE_DISTANCE
				const float point_distance = distance_to_origin<PointT>(in.points.at(current_idx));
		#else
				const float point_distance = in.points.at(current_idx).z;
		#endif
				value = 0; //point_distance;
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
				if(value > threshold_min && value < threshold_max) return is_depth_step(in.points.at(current_idx), in.points.at(idx), 1.3);
				return false;
		}
}
#endif
