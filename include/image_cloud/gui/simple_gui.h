/*
 * Simplegui.h
 *
 *  Created on: 17.01.2015
 *      Author: fnolden
 */

#include <iostream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS
#include <image_geometry/pinhole_camera_model.h>

// Own
#include <gui/filter_value.h>
#include <gui/kitti/camera.h>
#include <gui/kitti/tf.h>
#include <gui/filelist.h>

#ifndef SRC_CALIBRATION_SIMPLE_GUI_H_
#define SRC_CALIBRATION_SIMPLE_GUI_H_

namespace image_cloud {

enum Filter
{
	INTENSITY = 0,
	DEPTH = 1,
	DEPTH_INTENSITY = 2,
	HARRIS_3D = 3
};

struct Slider_data{
	int max;
	int val;
};



struct Set_selector{
	Filelist images;
	Filelist pointclouds;
	Slider_data pos;
	int pos_loaded;
};

struct Config_data
{
	std::string path;
	std::string image_file;
	std::string pcl_file;
	Filter filter;
	Slider_data filter_selector;
	Set_selector set;
};

class Simple_gui {

public:
	Simple_gui();
	virtual ~Simple_gui();
	void init();

	void load_image();
	bool load_pcl();
	void load_projection();

	void update_view();

	void filter3d();
	void project2image(cv::Mat &image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	void create_gui();
	void recreate_config_gui();
	void create_gui_filter();

	void update_values();
	void loop();
	void init_filter_data();
	void init_tf();

	std::string window_name;
	std::string window_name_transform;
	std::string window_name_control;


	Config_data data;

	cv::Mat image_file;
	cv::Mat image_display;
	boost::mutex filter_lock;

	char filterNames[4][50];
	std::vector<std::vector<Filter_value> >filter_data;
	Filter_value tf_data[6];

	image_geometry::PinholeCameraModel camera_model;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_file;
};

} /* namespace image_cloud */

#endif /* SRC_CALIBRATION_SIMPLE_GUI_H_ */
