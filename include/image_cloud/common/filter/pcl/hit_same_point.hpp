#ifndef FILTER_HIT_SAME_POINT_H_
#define FILTER_HIT_SAME_POINT_H_

#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/project2d.hpp>
#include <pcl/common/common.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

//#define DEBUG

namespace filter_3d{

	template <typename PointT>
	inline void
	hit_same_point(	const image_geometry::PinholeCameraModel &camera_model,
								const pcl::PointCloud<PointT> &in,
								pcl::PointCloud<PointT> &out,
								int rows,
								int cols,
								float z_threshold = 0.3)
	{
		std::vector<std::vector <std::vector<PointT> > > hit( cols, std::vector< std::vector<PointT> >(rows, std::vector<PointT>()));

		// Project points into image space
		for(unsigned int i=0; i < in.points.size();  ++i){
			const PointT* pt = &in.points.at(i);
			if( pt->z > 1) { // min distance from camera 1m

				cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(pt->x, pt->y, pt->z));

				if( between<int>(0, point_image.x, cols )
					&& between<int>( 0, point_image.y, rows )
				)
				{
					// Sort all points into image
					{
						hit[point_image.x][point_image.y].push_back(in.points.at(i));
					}

				}
			}
		}
		assert(out.empty());
		for(int x = 0; x < hit.size(); ++x ){
			for(int y = 0; y < hit[0].size(); ++y){
				if(hit[x][y].size()>1){
					PointT min_z = hit[x][y][0];
					float max_z = min_z.z;
					for(int p = 1; p < hit[x][y].size(); ++p){
					// find min and max z
						max_z = MAX(max_z, hit[x][y][p].z);
#ifdef DEBUG
						std::cout << hit[x][y].size() << "\t";
#endif

						if(hit[x][y][p].z < min_z.z)
						{
							min_z = hit[x][y][p];
						}
					}
#ifdef DEBUG
					std::cout << min_z.z << "\t" << max_z << "\t";
#endif
					if(max_z - min_z.z > z_threshold){
#ifdef DEBUG
						std::cout << min_z << std::endl;
#endif
						out.push_back(min_z);
					}
				}
			}
		}
#ifdef DEBUG
		std::cout << "hit_same_point in: "<< in.size()  << "\t out: " << out.size() << "\n";
#endif
	}
}
#endif
