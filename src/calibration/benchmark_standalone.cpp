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
#include <image_cloud/common/filter/cv/inverse_distance_transform.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>
#include <image_cloud/common/calibration/score.hpp>
#include <image_cloud/common/calibration/structs.hpp>
#include <image_cloud/common/calibration/grid_search.hpp>

void convert_to_grey(const cv::Mat& load, cv::Mat& grey) {
	// Convert Greyscale
	if (load.channels() != 1) {
		cvtColor(load, grey, CV_BGR2GRAY);
		assert(grey.channels() == 1);
	} else {
		load.copyTo(grey);
	}
}

//using namespace image_cloud;


int main(){
	printf("Starting \n");
	// Load kitti dataset
	kitti::Dataset data("/media/Daten/kitti/config_kitti_0005.txt");

	std::vector<cv::Mat> images;
	std::vector<pcl::PointCloud<pcl::PointXYZI> > pointclouds;
	std::vector<Projected_pointcloud<pcl::PointXYZI> > projected_pointclouds;

	int window_size = 5;
	int camera = 0;

	// Load camera model
	image_geometry::PinholeCameraModel camera_model;
	data.camera_list.cameras.at(camera).get_camera_model(camera_model);

	//Load and prepare data files;
	for(int i=0; i< window_size; ++i){
		// Load image
		cv::Mat load,grey,edge,inverse;

		data.camera_file_list.at(camera).load_image(load, i);
		printf("rows: %d, cols: %d\n", load.rows, load.cols);

		edge.create(load.rows, load.cols, CV_8U);
		inverse.create(load.rows, load.cols, CV_8U);

		// Convert Greyscale
		convert_to_grey(load, grey);

		// Edge filter
		filter_2d::edge_max<uchar>( grey, edge);

		// Inverse distance transform
		filter_2d::inverse_distance_transformation<uchar,uchar>(edge, inverse);

		// Put in array
		images.push_back( inverse);
		// Done with the image

		// Load pointcloud
		pcl::PointCloud<pcl::PointXYZI> read,transformed,filtred;
		std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > pointcloud_map( images[i].cols,
							std::vector<boost::shared_ptr<pcl::PointXYZI> > (images[i].rows));
		Projected_pointcloud<pcl::PointXYZI> pointcloud_projected;

		data.pointcloud_file_list.load_pointcloud(read, i);

		// Transforms velo_to_cam0
		tf::Transform velo_to_cam0;
		data.velodyne_to_cam0.get_transform(velo_to_cam0);

		// Transform cam0_to_cam
		tf::Transform cam0_to_cam;
		data.camera_list.cameras.at(camera).tf_rect.get_transform(cam0_to_cam);

		// Transform
		tf::Transform tf = (cam0_to_cam*velo_to_cam0);
		image_cloud::transform_pointcloud<pcl::PointXYZI>(read, transformed, tf);

		// To speed up search filter points here.
		project2d::project_2d(camera_model, transformed, pointcloud_map, pointcloud_projected, inverse.cols, inverse.rows);
		filter::filter_depth_intensity<pcl::PointXYZI>(pointcloud_map, filtred, 0.3, 50);

		pointclouds.push_back(filtred);
		printf("Filtred in: %lu projected: %lu depth_intesity: %lu\n", transformed.size(), pointcloud_projected.points.size(), filtred.size());
	}

	// Setup search
	search::Search_setup search_range;
	std::vector<search::Search_value> search_values;

	search_range.x.init_range(0,1,3);
	search_range.y.init_range(0,1,3);
	search_range.z.init_range(0,1,3);
	search_range.roll.init_range(0,1,3);
	search_range.pitch.init_range(0,1,3);
	search_range.yaw.init_range(0,1,3);

	//-0.5 y: -0.5 z: -0.5 roll: -0.5 pitch: 0.5 yaw: -0.5
	search::grid_setup(search_range, search_values);

	search::calculate<pcl::PointXYZI, uchar>(camera_model, pointclouds, images, search_values);

	int best_result_idx = 0;
	long unsigned int best_result = 0;
	for(int i=0; i< search_values.size(); ++i){
		if(search_values.at(i).result > best_result){
			best_result_idx = i;
			best_result = search_values.at(i).result;
		}
		printf("%d: \t%s\n", i, search_values.at(i).to_string().c_str());
	}

	printf("Best result with window size: %lu\n", images.size());
	printf("Setup: %s", search_range.to_string().c_str());
	printf("%d: \t%s\n",best_result_idx, search_values.at(best_result_idx).to_string().c_str());









	//image_cloud::Gui_opencv gui;
	//gui.init();

	return 0;
}


