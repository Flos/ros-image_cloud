#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>


#ifndef SCORE_H_
#define SCORE_H_

namespace score
{

void
score(
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
void
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

		int hit = 0;
		int points = 0;
		for(int y = 0; y < idx[0].size(); y++)
		{
			for(int x = 0; x < idx.size(); x++)
			{
				if( idx[x][y]){ /* found something */
					++points;
					if( image_edge.at<uchar>(x,y) > threshold){
						++hit;
					}
				}
			}
		}

		score = (float)hit/(float)points;
		printf("Points: %d, threshold %i, hits: %d, Score: %f\n", points, threshold, hit, score );
}


}
#endif
