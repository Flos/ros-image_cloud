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
#include <image_cloud/common/calibration/pipeline/image.hpp>
#include <image_cloud/common/calibration/pipeline/pointcloud.hpp>

void find_tf(std::string dataset, std::string output_file,
		tf::Transform bad_movement, tf::Transform search_startpoint,
		int min_steps, int max_steps, int max_repeats,
		float range_axis, float range_rotation,
		int window_size, int camera,
		bool pre_filtred) {
	// Load kitti dataset
	kitti::Dataset data(dataset);

	std::deque<cv::Mat> images;
	std::deque<pcl::PointCloud<pcl::PointXYZI> > pointclouds;
	std::vector<Projected_pointcloud<pcl::PointXYZI> > projected_pointclouds;

	// Load camera model
	image_geometry::PinholeCameraModel camera_model;
	data.camera_list.cameras.at(camera).get_camera_model(camera_model);
	//Load and prepare data files;
	for (int i = 0; i < window_size; ++i) {
		// Load image
		cv::Mat imgage_load, image_inverse;
		data.camera_file_list.at(camera).load_image(imgage_load, i);
		// Grey, edge, inverse transformation;
		image_cloud::create_inverse_transformed(imgage_load, image_inverse);
		// Put in array
		images.push_back(image_inverse);
		// Done with the image
		// Calculate TF
		// Transforms velo_to_cam0
		tf::Transform velo_to_cam0;
		data.velodyne_to_cam0.get_transform(velo_to_cam0);
		// Transform cam0_to_cam
		tf::Transform cam0_to_cam;
		data.camera_list.cameras.at(camera).tf_rect.get_transform(cam0_to_cam);
		// Transform
		tf::Transform tf = (cam0_to_cam * velo_to_cam0 * bad_movement);
		// Load pointcloud
		pcl::PointCloud<pcl::PointXYZI> read, transformed, filtred;
		std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > pointcloud_map(
				images[i].cols,
				std::vector<boost::shared_ptr<pcl::PointXYZI> >(
						images[i].rows));
		data.pointcloud_file_list.load_pointcloud(read, i);
		image_cloud::transform_pointcloud<pcl::PointXYZI>(read, transformed,
				tf);
		if (pre_filtred) {
			Projected_pointcloud<pcl::PointXYZI> pointcloud_projected;
			// To speed up search filter points here.
			project2d::project_2d(camera_model, transformed, pointcloud_map,
					pointcloud_projected, image_inverse.cols,
					image_inverse.rows);
			filter::filter_depth_intensity<pcl::PointXYZI>(pointcloud_map,
					filtred, 0.3, 50);
			pointclouds.push_back(filtred);
			printf(
					"Filtred search_startpoint: %lu projected: %lu depth_intesity: %lu\n",
					transformed.size(), pointcloud_projected.points.size(),
					filtred.size());
		} else {
			pointclouds.push_back(transformed);
		}
	}
	//Benchmark processing time
	std::string spacer = " \t";
	std::stringstream sum;
	std::ofstream myfile;
	myfile.open(output_file.c_str());

	sum << "steps" << spacer;
	sum << "calculations" << spacer;
	sum << "repeats" << spacer;
	sum << "time_total_sec" << spacer;
	sum << "time_total_nsec" << spacer;
	sum << "time_single_sec" << spacer;
	sum << "time_single_nsec" << spacer;
	sum << "range_axis" << spacer;
	sum << "range_rotation" << spacer;
	sum << "prefiltered" << spacer;
	sum << "best_result\n";

	myfile << sum.str();
	std::cout << sum.str();

	for (int i = min_steps; i <= max_steps; ++i) {
		// Preparation:
		std::stringstream ss;
		tf::Transform out;
		search::Multi_search_result result;
		ros::WallTime time1 = ros::WallTime::now();
		for (int repeats = 0; repeats < max_repeats; ++repeats) {
			search::get_best_tf<pcl::PointXYZI, uchar>(search_startpoint, out,
					camera_model, pointclouds, images, range_axis,
					range_rotation, i, pre_filtred, &result);
		}
		ros::WallTime time2 = ros::WallTime::now();
		ros::WallTime single_it;
		ros::WallDuration delta;
		delta = (time2 - time1);
		single_it.sec = delta.sec / max_repeats;
		single_it.nsec = delta.nsec / max_repeats;
		ss << i << spacer << result.nr_total << spacer;
		ss << max_repeats << spacer;
		ss << delta.sec << spacer;
		ss << delta.nsec << spacer;
		ss << single_it.sec << spacer;
		ss << single_it.nsec << spacer;
		ss << range_axis << spacer;
		ss << range_rotation << spacer;
		ss << pre_filtred << spacer;
		ss << result.to_string() << std::endl;
		std::cout << result.in.to_string() << "\n";
		std::cout << result.best.to_string() << "\n";
		myfile << ss.str();
		sum << ss;
	}
	myfile.close();
}

//using namespace image_cloud;


int main(){
	tf::Transform search_startpoint, bad_movement;
	bad_movement.setIdentity();
	search_startpoint.setIdentity();

	// Config;
	std::string dataset = "/media/Daten/kitti/config_kitti_0005.txt";
	std::string output_file = "pre_filtred.txt";

	int min_steps = 3;
	int max_steps = 5;
	int max_repeats = 100;
	float range_axis = 0.04;
	float range_rotation = 0.04;
	bool pre_filtred = true;
	int window_size = 5;
	int camera = 0;

	bad_movement.setOrigin(tf::Vector3(0.03, 0.03, 0.03));
	//search_startpoint.setOrigin(tf::Vector3(0.02, 0.02, 0.02));

	find_tf(dataset, "filtred1.txt"	, bad_movement, search_startpoint, min_steps, max_steps, max_repeats, range_axis, range_rotation, window_size, camera, pre_filtred);
	find_tf(dataset, "filtred2.txt"	, bad_movement, search_startpoint, min_steps, max_steps, max_repeats, range_axis, range_rotation, window_size, camera, pre_filtred);

	pre_filtred = false;
	find_tf(dataset, "unfiltred1.txt"	, bad_movement, search_startpoint, min_steps, max_steps, max_repeats, range_axis, range_rotation, window_size, camera, pre_filtred);
	find_tf(dataset, "unfiltred2.txt"	, bad_movement, search_startpoint, min_steps, max_steps, max_repeats, range_axis, range_rotation, window_size, camera, pre_filtred);


	return 0;
}


