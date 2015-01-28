#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <image_geometry/pinhole_camera_model.h>

// kitti
#include <kitti/common/serialization/dataset.h>
//#include <image_cloud/common/kitti/camera.h>
//#include <image_cloud/common/kitti/tf.h>
//#include <image_cloud/common/string_list.h>
//#include <image_cloud/common/kitti/dataset.h>

// Own hpp
#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/score.hpp>
#include <image_cloud/common/filter/cv/inverse_distance_transform.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>



//using namespace image_cloud;


int main(){
	printf("Starting \n");

	// Load kitti dataset
	kitti::Dataset data("/media/Daten/kitti/config_kitti_0005.txt");


	//image_cloud::Gui_opencv gui;
	//gui.init();

	return 0;
}


