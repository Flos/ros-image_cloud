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

//own
#include <gui/kitti/serializable.h>

#ifndef INCLUDE_KITTI_CAMERA_H_
#define INCLUDE_KITTI_CAMERA_H_

namespace image_cloud {

namespace kitti{

class Camera : public Serializable{
public:
	int camera_nr;
	float S[2];
	float K[9];
	float D[5];
	float R[9];
	float T[3];
	float S_rect[2];
	float R_rect[9];
	float P_rect[12];
	void get_camera_info(sensor_msgs::CameraInfo &info_msg, bool rect = true);
	void set_camera_info(sensor_msgs::CameraInfo info_msg, bool rect = true);

	Camera();
	Camera(int camera_nr);
	std::string to_string();
	bool load( std::istream& stream);
};

}

} /* namespace image_cloud */

#endif /* INCLUDE_KITTI_CAMERA_H_ */
