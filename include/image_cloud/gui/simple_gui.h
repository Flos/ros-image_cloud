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

struct Config_data
{
	std::string path;
	std::string image_file;
	std::string pcl_file;
	Filter filter;
};

struct Slider_data{
	int filter_max;
	int filter;
};

struct Slider_tf
{
	int val_max;
	int translation_scale;
	int rotation_scale;

	int tx;
	int ty;
	int tz;

	int roll;
	int pitch;
	int yaw;
};


struct Config_transfrom
{
	float tx;
	float ty;
	float tz;

	float roll;
	float pitch;
	float yaw;
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
	void update_tf();
	void loop();
	void init_filter_data();

	std::string window_name;
	std::string window_name_control;

	Config_data data;
	Config_transfrom tf_data;
	Slider_tf tf_slider;
	Slider_data data_slider;



	cv::Mat image_file;
	cv::Mat image_display;
	boost::mutex filter_lock;

	char filterNames[4][50];
	std::vector<std::vector<Filter_value> >filter_data;

	image_geometry::PinholeCameraModel camera_model;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_file;
};

} /* namespace image_cloud */

#endif /* SRC_CALIBRATION_SIMPLE_GUI_H_ */
