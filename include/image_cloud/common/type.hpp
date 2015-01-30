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

	std::string to_string(){
		std::stringstream ss;
			ss << "pcl: " << pcl << "cv: " << cv;
		return ss.str();
	}
};

struct Image_size{
	int width;
	int height;
};

template <typename PointT>
struct Projected_pointcloud{
	std::vector<Projected_Point<PointT> > points;
	Image_size image_size;

	std::string to_string(){
		std::stringstream ss;
		ss << "width: " << image_size.width << " height: " << image_size.height << "\n";
		for(int i = 0; i < points.size(); ++i){
			ss << "i: " << i << " " << points.at(i).to_string() << "\n";
		}
		return ss.str();
	}

	size_t size(){
		return points.size();
	}

	Projected_Point<PointT> at(long unsigned int i){
		return points.at(i);
	}

	void get_pointcloud(pcl::PointCloud<PointT> &out){
		assert(out.points.empty());

		for(int i = 0; i < size(); ++i){
			out.push_back(points.at(i).pcl);
		}
	}


};

#endif /* MY_TEMPLATE_TYPES_H_ */
