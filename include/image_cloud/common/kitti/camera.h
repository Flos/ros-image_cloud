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
#include <image_cloud/common/kitti/serializable.h>
#include <image_cloud/common/kitti/tf.h>

#ifndef INCLUDE_KITTI_CAMERA_H_
#define INCLUDE_KITTI_CAMERA_H_

namespace image_cloud {

namespace kitti{

class Camera : public Serializable{
public:
	std::string name;
	std::string frame_id;
	std::string id;
	int camera_nr;
	float S[2]; // original image size
	float K[9]; // calibration matrices (unrectified)
	float D[5]; // distortion coefficients (unrectified)
	float S_rect[2]; // image size after rectification
	float P_rect[12]; // projection matrix after rectification

	// rectifying rotation matrix
	Tf tf; // rotation from camera 0 to camera i, translation from camera 0 to camera i
	Tf tf_rect; // rotation from camera 0 to camera i, translation from camera 0 to camera i

	void get_camera_info(sensor_msgs::CameraInfo &info_msg);
	void set_camera_info(sensor_msgs::CameraInfo info_msg);

	void get_projection(Tf &tf);

	Camera(int camera_nr = 0, std::string name = "camera");
	void set_camera_nr(int camera);

	std::string to_string();
	bool load( std::istream& stream);
};

}

} /* namespace image_cloud */

#endif /* INCLUDE_KITTI_CAMERA_H_ */
