#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <common/type.hpp>


#ifndef SCORE_H_
#define SCORE_H_

namespace score
{

void
inline score(
		const pcl::PointCloud<pcl::PointXY> &in,
		cv::Mat &image_edge,
		float &score,
		int threashold = 30
		)
{
		assert(image_edge.channels() == 1);

		int hit = 0;

		for(int i = 0; i < in.size(); ++i)
		{
				if( image_edge.at<uchar>( in.at(i).x, in.at(i).y) > threashold){
					++hit;
				}
		}
		score = hit/in.size();
		printf("Score: %f  hits: %d, threshold %i", score, hit, threashold );
}


template <typename PointT>
inline void
score(
		std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
		cv::Mat &image_edge,
		float &score,
		int threshold = 30
		)
{

		assert(image_edge.channels() == 1);
		assert(image_edge.depth() == CV_8U  || image_edge.depth() == CV_8S);
		assert(image_edge.rows == idx[0].size());
		assert(image_edge.cols == idx.size());
		score = 0;

		long unsigned int hit = 0;
		long unsigned int points = 0;
		long unsigned int pixels_with_value = 0;

		for(int y = 0; y < idx[0].size(); y++)
		{
			for(int x = 0; x < idx.size(); x++)
			{
				int pixel_value = image_edge.at<uchar>(x,y);
				if( idx[x][y]){ /* found something */
					++points;
					if( pixel_value > threshold){
						++hit;
					}
				}
				else if(pixel_value != 0){
					++pixels_with_value;
				}
			}
		}

		if(pixels_with_value > (image_edge.rows*image_edge.cols*0.3)){
			printf("Error no proper image with features applied\n");
			score = 0;
		}
		else{
			// Should be high when many points hit.
			// but should be low if chance to hit is to high
			score = (float)hit/(float)points;
		}
		printf("Pixels: %lu, Points: %lu, threshold %i, hits: %lu, Score: %f\n", pixels_with_value, points, threshold, hit, score );
}

template <typename PointT, typename ImageT>
inline void
objective_function(
		std::vector<std::vector<boost::shared_ptr<PointT> > > &idx,
		cv::Mat &image_edge,
		long unsigned &score)
{

		assert(cv::DataType<ImageT>::channels == 1);
		assert(image_edge.rows == idx[0].size());
		assert(image_edge.cols == idx.size());

		score = 0;

		for(int y = 0; y < idx[0].size(); y++)
		{
			for(int x = 0; x < idx.size(); x++)
			{
				if( idx[x][y]){ /* found something */
					score += image_edge.at<ImageT>(x,y);
				}
			}
		}

}

template <typename PointT, typename ImageT>
inline void
objective_function(
		Projected_Pointclouds<PointT> &idx,
		cv::Mat &image_edge,
		long unsigned &score)
{

		assert(cv::DataType<ImageT>::channels == 1);
		assert(image_edge.rows == idx.image_size.width);
		assert(image_edge.cols == idx.image_size.height);

		score = 0;

		for(int i = 0; i < idx.points.size(); i++)
		{
			score += image_edge.at<ImageT>(idx.points.cv);
		}

}



}
#endif
