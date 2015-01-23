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
#include <common/kitti/camera.h>
#include <common/kitti/tf.h>
#include <common/string_list.h>
#include <common/kitti/dataset.h>
#include <gui/slider.h>

#ifndef SRC_CALIBRATION_SIMPLE_GUI_H_
#define SRC_CALIBRATION_SIMPLE_GUI_H_

namespace image_cloud {

namespace pcl_filter{
	enum Filter3d
	{
		OFF = 0,
		DEPTH = 1,
		DEPTH_INTENSITY = 2,
		OTHER = 3
	};
}

namespace image_filter{

	namespace blur{
		enum Blur{
			OFF = 0,
			BILATERAL = 1,
			BLUR = 2,
			GAUSSIAN = 3,
			MEDIAN = 4
		};
	}

	namespace edge{
		enum Edge{
			OFF = 0,
			CANNY = 1,
			LAPLACE = 2
		};
	}

	namespace enlight{
		enum Enlight{
			OFF = 0
		};
	}

	enum image_index{
		FILE_READ = 0,
		IMAGE_GREY = 1,
		IMAGE_BLUR = 2,
		IMAGE_EDGE = 3,
		IMAGE_FULL = 4,
		IMAGE_POINTS = 5
	};

	struct Image_filter{
		Slider blur;
		std::vector< std::vector<Slider> >blur_values;
		Slider edge;
		std::vector<std::vector<Slider> >edge_values;
		std::string window_name;
	};

}

struct Set_selector{
	String_list images;
	String_list pointclouds;
	Slider pos;
};



struct Dataset_config{
	Slider pos_image;
	Slider pos_camera;
};

//Date for all
struct Datasets_list{
	std::vector<kitti::Dataset> list_datasets;
	std::vector<Dataset_config> list_config;
	std::vector<std::vector<Slider> >filter3d_data;
	image_filter::Image_filter filter2d;
	Slider pos_dataset;
	Slider processed_image_selector;
	Slider projection;
	Slider pcl_filter;
};

class Gui_opencv {

public:
	Gui_opencv();
	virtual ~Gui_opencv();
	void init();

	void load_image();
	bool load_pcl();
	void load_projection();

	void update_view();

	void filter3d();
	void filter2d();
	void project2image(cv::Mat &image, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
	void create_gui_general_conf();
	void recreate_config_gui();
	void create_gui_filter2d();
	void create_gui_filter3d();

	void update_values();
	void update_image();
	void loop();
	void init_menu_options();
	void init_tf();
	void create_gui_manual_tf();
	void init_datasets();

	std::string window_name;
	std::string window_name_transform;
	std::string window_name_general_conf;

	Datasets_list datasets;
	std::vector<std::string> config_files;

	std::vector<cv::Mat> images;
	boost::mutex filter_lock;

	// config common
	std::vector<std::string> filter3d_names;
	std::vector<std::string> filter2d_blur_names;
	std::vector<std::string> filter2d_edge_names;
	//std::vector<std::vector<Filter_value> >filter_data;
	Slider tf_data[6];

	image_geometry::PinholeCameraModel camera_model;
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_file;
};

} /* namespace image_cloud */

#endif /* SRC_CALIBRATION_SIMPLE_GUI_H_ */
