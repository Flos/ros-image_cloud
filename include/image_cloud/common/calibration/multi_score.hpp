#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <common/type.hpp>
#include <math.h>

#ifndef MUTLI_SCORE_H_
#define MUTLI_SCORE_H_

namespace score
{



template <typename PointT, typename ImageT>
inline void
multi_score(
		std::vector<Projected_Pointclouds<PointT> > &idx,
		std::vector<cv::Mat> &edge_images,
		long unsigned &score)
{
	assert(idx.size() == edge_images.size());
	for(int i = 0; i< idx.size(); ++i){
		long unsigned score_temp;
		objective_function(idx.at(i), edge_images.at(i),score_temp);
		score += score_temp;
	}

}


}

#endif
