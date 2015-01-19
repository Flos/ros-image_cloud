/*
 * kitti_camera.h
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

#ifndef INCLUDE_IMAGE_CLOUD_GUI_KITTI_CAMERA_H_
#define INCLUDE_IMAGE_CLOUD_GUI_KITTI_CAMERA_H_

namespace image_cloud {

namespace kitti{

class camera{
public:
	float S[2];
	float K[9];
	float D[5];
	float R[9];
	float T[3];
	float S_rect[2];
	float R_rect[9];
	float P_rect[12];
	std::string save(int camera_nr);
	bool load(std::ifstream &file);
	void get_camera_info(sensor_msgs::CameraInfo &info_msg, bool rect = true);
	void set_camera_info(sensor_msgs::CameraInfo info_msg, bool rect = true);
private:
	void serialize_array(std::stringstream &ss, float*, int array_size);
	void deserialize_array(std::istringstream &in, float*, int array_size);
};

class velo_to_cam{
public:
	float R[9];
	float T[3];
	float delta_f[2];
	float delta_c[2];
	void get_transform(tf::Transform &tf);
	void set_transform(tf::Transform tf);
};

}

} /* namespace image_cloud */

#endif /* INCLUDE_IMAGE_CLOUD_GUI_KITTI_CAMERA_H_ */
