#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>

#ifndef MY_TEMPLATE_TYPES_H_
#define MY_TEMPLATE_TYPES_H_

template <typename PointT>
struct Projected_Point{
	Projected_Point(PointT pcl, cv::Point2i cv){
		this->pcl = pcl;
		this->cv = cv;
	}
	PointT pcl;
	cv::Point2i cv;
};

struct Image_size{
	int width;
	int height;
};
template <typename PointT>
struct Projected_Pointclouds{
	std::vector<Projected_Point<PointT> > points;
	Image_size image_size;
};

#endif /* MY_TEMPLATE_TYPES_H_ */
