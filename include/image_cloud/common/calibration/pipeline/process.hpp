#ifndef IMAGE_CLOUD_COMMON_PIPELINE_PROCESS_H_
#define IMAGE_CLOUD_COMMON_PIPELINE_PROCESS_H_

#include <image_cloud/common/calibration/pipeline/enums.h>
#include <image_cloud/common/calibration/pipeline/image.hpp>
#include <image_cloud/common/calibration/pipeline/pointcloud.hpp>

namespace image_cloud{

template <typename PointT>
inline void
process( const pcl::PointCloud<PointT> &in, image_geometry::PinholeCameraModel &camera_model, cv::Mat &out){
	//project2d::
}



#endif
