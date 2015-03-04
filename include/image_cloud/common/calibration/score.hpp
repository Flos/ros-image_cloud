#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <pcl/common/common.h>
#include <opencv2/core/core.hpp>

#ifndef SCORE_H_
#define SCORE_H_

//#define WEIGHT_INTENSITY

namespace score
{
//
//void
//inline score(
//		const pcl::PointCloud<pcl::PointXY> &in,
//		cv::Mat &image_edge,
//		float &score,
//		int threashold = 30
//		)
//{
//		assert(image_edge.channels() == 1);
//
//		int hit = 0;
//
//		for(int i = 0; i < in.size(); ++i)
//		{
//				if( image_edge.at<uchar>( in.at(i).x, in.at(i).y) > threashold){
//					++hit;
//				}
//		}
//		score = hit/in.size();
//		printf("Score: %f  hits: %d, threshold %i", score, hit, threashold );
//}
//
//
//template <typename PointT>
//inline void
//score(
//		const std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
//		const cv::Mat &image_edge,
//		float &score,
//		int threshold = 30
//		)
//{
//
//		assert(image_edge.channels() == 1);
//		assert(image_edge.depth() == CV_8U  || image_edge.depth() == CV_8S);
//		assert(image_edge.rows == idx[0].size());
//		assert(image_edge.cols == idx.size());
//		score = 0;
//
//		long unsigned int hit = 0;
//		long unsigned int points = 0;
//		long unsigned int pixels_with_value = 0;
//
//		for(int y = 0; y < idx[0].size(); y++)
//		{
//			for(int x = 0; x < idx.size(); x++)
//			{
//				int pixel_value = image_edge.at<uchar>(x,y);
//				if( idx[x][y]){ /* found something */
//					++points;
//					if( pixel_value > threshold){
//						++hit;
//					}
//				}
//				else if(pixel_value != 0){
//					++pixels_with_value;
//				}
//			}
//		}
//
//		if(pixels_with_value > (image_edge.rows*image_edge.cols*0.3)){
//			printf("Error no proper image with features applied\n");
//			score = 0;
//		}
//		else{
//			// Should be high when many points hit.
//			// but should be low if chance to hit is to high
//			score = (float)hit/(float)points;
//		}
//		printf("Pixels: %lu, Points: %lu, threshold %i, hits: %lu, Score: %f\n", pixels_with_value, points, threshold, hit, score );
//}


//template <typename PointT, typename ImageT>
//inline void
//objective_function(
//		const Projected_pointcloud<PointT> &idx,
//		const cv::Mat &image_edge,
//		long unsigned &score)
//{
//		assert(cv::DataType<ImageT>::channels == 1);
//		assert(image_edge.cols == idx.image_size.width);
//		assert(image_edge.rows == idx.image_size.height);
//
//		cv::Mat temp;
//		image_edge.copyTo(temp);
//
//		score = 0;
//		for(int i = 0; i < idx.points.size(); i++)
//		{
//			score += temp.at<ImageT>(idx.points.at(i).cv);
//			temp.at<ImageT>(idx.points.at(i).cv) = 0; // no higher score if multiple points hit the same edge
//		}
//}

template <typename PointT, typename ImageT>
inline void
objective_function(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<PointT> &in,
		const cv::Mat &image_edge,
		long unsigned &score,
		bool use_intensity_as_weight=false)
{
		cv::Mat temp;
		image_edge.copyTo(temp);

		score = 0;
		for(int i = 0; i < in.size(); ++i ){
			if( in.points.at(i).z > 1) { // min distance from camera 1m

				cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

				if( between<int>(0, point_image.x, image_edge.cols )
					&& between<int>( 0, point_image.y, image_edge.rows )
				)
				{
					if(use_intensity_as_weight){
						score += temp.at<ImageT>(point_image)*(in.points.at(i).intensity);
					}
					else{
						score += temp.at<ImageT>(point_image);
					}
					temp.at<ImageT>(point_image) = 0; // no higher score if multiple points hit the same edge
				}
			}
		}
}

}
#endif
