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

//
#include <tf/tf.h>

//ROS
#include <image_geometry/pinhole_camera_model.h>

//own
#include <gui/kitti/serializable.h>

#ifndef INCLUDE_KITTI_TF_H_
#define INCLUDE_KITTI_TF_H_

namespace image_cloud {

namespace kitti{

class Tf : public Serializable {
public:
	Tf();
	float R[9];
	float T[3];
	float delta_f[2];
	float delta_c[2];
	void get_transform(tf::Transform &tf);
	void set_transform(tf::Transform tf);
	std::string to_string();
	bool load( std::istream &stream);
};

}

} /* namespace image_cloud */

#endif /* INCLUDE_KITTI_TF_H_ */
