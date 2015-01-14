#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>

#include <common/small_helpers.hpp>

#ifndef PCL_DEPTH_FILTER_H_
#define PCL_DEPTH_FILTER_H_

inline void
extract_depth_discontinuity(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<pcl::PointXYZI>::Ptr in,
		cv_bridge::CvImage& image_pcl,
		int point_size = 1
)
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n( new pcl::search::KdTree<pcl::PointXYZI>() );
	tree_n->setInputCloud(in);
	tree_n->setEpsilon(0.5);

	unsigned int hits = 0;
	unsigned int hits_image = 0;

	BOOST_FOREACH (const pcl::PointXYZI& pt, in->points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		cv::Point2d point_image;
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
					&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
					&& point_image.y < image_pcl.image.rows )
			)
			{
				hits_image++;
				std::vector<int> k_indices;
				std::vector<float> square_distance;
				int neighbors = 0;

				//Position in image is known
				neighbors = tree_n->nearestKSearch(pt, 2, k_indices, square_distance);

				// look for neighbors where at least 1 is closer (Distance to origin) than the other
				float distance_selected = sqrt(/*(pt.x* pt.x)+(pt.y* pt.y)+*/(pt.z* pt.z));
				float intensity_max = pt.intensity ;

				if (neighbors > 0) {
					int nearest_neighbor = 0;

					for(int i = 0; i < neighbors; i++){
						if( intensity_max < in->points.at(k_indices.at(i)).intensity ) intensity_max = in->points.at(k_indices.at(i)).intensity;


						float current_distance =  //(in->points.at(k_indices.at(i)).x* in->points.at(k_indices.at(i)).x)
												 // +(in->points.at(k_indices.at(i)).y* in->points.at(k_indices.at(i)).y)
												  +(in->points.at(k_indices.at(i)).z* in->points.at(k_indices.at(i)).z);
						current_distance = sqrt(current_distance);

						float range = 0.15;

						if( 	(pt.z - range) < in->points.at(k_indices.at(i)).z
							&& 	in->points.at(k_indices.at(i)).z > (pt.z + range)
								)
						{
							//same area.. dont do anything
							cv::circle(image_pcl.image, point_image, point_size, cv::Scalar(intensity_max), -1);
														hits++;
														break;
						}
						else{

						}
						// 1. Is one of the neighbors closer than selected?
//						if( sqrt((distance_selected - current_distance)*(distance_selected - current_distance)) > 0.3 ){
//							// YES
//							cv::circle(image_pcl.image, point_image, point_size, cv::Scalar(intensity_max), -1);
//							//out->push_back(pt);
//							hits++;
//							break;
//						}
					}
				}
			}
		}
	}
	ROS_INFO("Hits: %d Hit_image: %d cloud: %d", hits, hits_image, in->size());
}


inline void
filter_depth_discontinuity(
		pcl::PointCloud<pcl::PointXYZI>::Ptr in,
		pcl::PointCloud<pcl::PointXYZI>::Ptr out,
		int neighbohrs_to_search = 2,
		float epsilon = 0.5
)
{
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n( new pcl::search::KdTree<pcl::PointXYZI>() );
	tree_n->setInputCloud(in);
	tree_n->setEpsilon(epsilon);

	BOOST_FOREACH (const pcl::PointXYZI& pt, in->points){
		std::vector<int> k_indices;
		std::vector<float> square_distance;
		int found_neighbors = 0;

		//Position in image is known
		found_neighbors = tree_n->nearestKSearch(pt, neighbohrs_to_search, k_indices, square_distance);

		// look for neighbors where at least 1 is closer (Distance to origin) than the other

		if (found_neighbors > 0) {
			int nearest_neighbor = 0;
			float distance = in->points.at(k_indices.at(0)).z;
			for(int i = 1; i < found_neighbors; i++){

				float current_distance = in->points.at(k_indices.at(i)).z;

				// 1. Is one of the neighbors closer than selected?
				if(distance > current_distance){
					// YES
					out->push_back(pt);
					break;
				}
			}
		}
	}
}

inline void filter_depth_edges(
		pcl::PointCloud<pcl::PointXY>::Ptr in_projected_points,
		pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_points,
		cv_bridge::CvImage& image_pcl,
		int neighbohrs_to_search = 2,
		int point_size = 1
)
{
	assert( in_projected_points->points.size() == in_origin_points->points.size());

	pcl::search::KdTree<pcl::PointXY>::Ptr tree_n( new pcl::search::KdTree<pcl::PointXY>() );
	tree_n->setInputCloud(in_projected_points);
	tree_n->setEpsilon(0.5);

	unsigned int hits = 0;
	unsigned int i_p = 0;
	float cloud_intensity_min = 9999;
	float cloud_intensity_max = 0;

	BOOST_FOREACH (const pcl::PointXY& pt, in_projected_points->points){
		pcl::PointXYZI* pt_ori = &in_origin_points->points.at(i_p);
		++i_p;

		std::vector<int> k_indices;
		std::vector<float> square_distance;
		int found_neighbors = 0;

		//Position in image is known
		found_neighbors = tree_n->nearestKSearch(pt, neighbohrs_to_search, k_indices, square_distance);
		//found_neighbors = tree_n->radiusSearch(pt, 2, k_indices, square_distance);

		// look for neighbors where at least 1 is closer (Distance to origin) than the other
		float pt_depth = pt_ori->z;
		//float distance_selected = point_distance(*pt_ori);
		float intensity_max = pt_ori->intensity;

		if(intensity_max < cloud_intensity_min) cloud_intensity_min = intensity_max;
		if(intensity_max > cloud_intensity_max) cloud_intensity_max = intensity_max;

		int valid = 0;

		if (found_neighbors > 0) {

			int nearest_neighbor = 0;

			for(int i = 0; i < found_neighbors; i++){

				float intensity = in_origin_points->points.at(k_indices.at(i)).intensity;

				if(intensity > intensity_max) intensity_max = intensity;

				//float current_distance = point_distance(in_origin_points->points.at(k_indices.at(i)));
				float current_distance = in_origin_points->points.at(k_indices.at(i)).z;

				// 1. Is one of the neighbors closer than selected?
				if( pt_depth - current_distance  >  0.5){
					++valid;
				}
				if( !inRange(intensity, 30.0, pt_ori->intensity)){
					++valid;
				}
			}

		}
		if(valid > 0){
			cv::circle(image_pcl.image, cv::Point2f(pt.x, pt.y), point_size, cv::Scalar(intensity_max), -1);
			hits++;
		}
	}
	ROS_INFO("depth cloud: %d, hits: %d imin: %f imax: %f", in_projected_points->size(), hits, cloud_intensity_min, cloud_intensity_max);
}


inline void filter_depth_edges_easy(
		pcl::PointCloud<pcl::PointXY>::Ptr in_projected_points,
		pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_points,
		cv_bridge::CvImage& image_pcl,
		int neighbohrs_to_search = 2,
		int point_size = 1
)
{
	assert( in_projected_points->points.size() == in_origin_points->points.size());

	unsigned int hits = 0;
	unsigned int i_p = 0;

	BOOST_FOREACH (const pcl::PointXY& pt, in_projected_points->points){
		pcl::PointXYZI* pt_ori = &in_origin_points->points.at(i_p);
		++i_p;

		// look for neighbors where at least 1 is closer (Distance to origin) than the other
		//float distance_selected = pt_ori->z;
		float distance_selected = point_distance(*pt_ori);
		float intensity_max = pt_ori->intensity ;

		if (i_p < in_origin_points->points.size() && i_p < 1  ) {

			bool valid = false;

			for(int i = 0; i < neighbohrs_to_search; i++){
				int idx = i_p - 2 + i;
				if(in_origin_points->points.at(idx).intensity > intensity_max) intensity_max = in_origin_points->points.at(idx).intensity;

				float current_distance = point_distance(in_origin_points->points.at(idx));
				//float current_distance = in_origin_points->points.at(k_indices.at(i)).z;

				// 1. Is one of the neighbors closer than selected?
				if( sqrt((distance_selected - current_distance)*(distance_selected - current_distance)) > 0.3){
					valid = true;
				}
			}

			if(valid){
				cv::circle(image_pcl.image, cv::Point2f(pt.x, pt.y), point_size, cv::Scalar(intensity_max), -1);
				hits++;
			}

		}
	}
	ROS_INFO("easy cloud: %d, hits: %d", in_projected_points->size(), hits);
}

#endif
