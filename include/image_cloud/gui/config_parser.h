/*
 * config_parser.h
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <time.h>
#include <stdio.h>
#include <assert.h>
#include <vector>

//
#include <tf/tf.h>

//ROS
#include <image_geometry/pinhole_camera_model.h>

//own
#include <gui/kitti_camera.h>

#ifndef SRC_GUI_CONFIG_PARSER_H_
#define SRC_GUI_CONFIG_PARSER_H_

namespace image_cloud {

class Config_parser {
public:
	Config_parser();
	virtual ~Config_parser();
	void read_camera();
	void write_camera_info();

	bool write_calib_velo_to_cam(std::string filename, kitti::velo_to_cam tf );
	bool load_calib_velo_to_cam(std::string filename, kitti::velo_to_cam &tf );

	void write_array(std::ofstream &stream, float* array, int size);
	void read_array(std::istringstream &line, float* array, int size);

	bool load_calib_cam_to_cam(std::string filename, float &corner_distance, std::vector<kitti::camera> &cams);
	bool write_calib_cam_to_cam(std::string filename, float corner_distance, std::vector<kitti::camera> &cams);
	void test();

	const std::string get_current_date_time();

	std::string def;
	std::string limiter;
	std::string new_line;
};

} /* namespace image_cloud */

#endif /* SRC_GUI_CONFIG_PARSER_H_ */
