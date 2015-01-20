/*
 * kitti_camera.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "gui/kitti/camera.h"

namespace image_cloud {

namespace kitti{

Camera::Camera()
{
	camera_nr = 0;
	set_zero(S, 2);
	set_zero(K, 9);
	set_zero(D, 5);

	set_zero(tf.R, 9);
	set_zero(tf.T, 3);

	set_zero(S_rect, 2);
	set_zero(tf_rect.R, 9);
	set_zero(tf_rect.T, 3);

	set_zero(P_rect, 12);
}

Camera::Camera(int camera_nr)
{
	this->camera_nr = camera_nr;
}

std::string
Camera::to_string(){
	std::stringstream ss;

	char buffer[3];
	sprintf(buffer,"%02d:", camera_nr);

	ss.setf(std::ios::scientific);
	ss << "S_" <<  buffer; serialize_array(ss, S, 2);
	ss << "K_" <<  buffer; serialize_array(ss, K, 9);
	ss << "D_" <<  buffer; serialize_array(ss, D, 5);
	ss << "R_" <<  buffer; serialize_array(ss, tf.R, 9);
	ss << "T_" <<  buffer; serialize_array(ss, tf.T, 3);
	ss << "S_rect_" <<  buffer; serialize_array(ss, S_rect, 2);
	ss << "R_rect_" <<  buffer; serialize_array(ss, tf_rect.R, 9);
	ss << "P_rect_" <<  buffer; serialize_array(ss, P_rect, 12);

	return ss.str();
}

void
Camera::get_camera_info(sensor_msgs::CameraInfo &info_msg){

	assert(info_msg.D.empty());
	assert(info_msg.K.size() == 9);
	assert(info_msg.P.size() == 12);

	for(int i = 0; i < info_msg.D.size(); ++i){
		info_msg.D.push_back(D[i]);
	}

	for(int i = 0; i < info_msg.K.size(); ++i){
		info_msg.K[i] = K[i];
	}

	for(int i = 0; i < info_msg.P.size(); ++i){
		info_msg.P[i] = P_rect[i];
	}
}

void
Camera::set_camera_info(sensor_msgs::CameraInfo info_msg){

	assert(info_msg.D.size() <= 5);
	assert(info_msg.K.size() == 9);
	assert(info_msg.P.size() == 12);

	for(int i = 0; i < info_msg.D.size(); ++i){
		D[i] = info_msg.D.at(i);
	}

	for(int i = 0; i < info_msg.K.size(); ++i){
		K[i] = info_msg.K[i];
	}

	for(int i = 0; i < info_msg.P.size(); ++i){
		P_rect[i] = info_msg.P[i];
	}
}

bool
Camera::load(std::istream& stream){
	std::string line;
	std::istringstream in;
	std::string type;

	try{
		// S 2
		if(std::getline (stream, line) <= 0) return false;
		in.str(line);
		in >> type;
		deserialize_array(in, S, 2);

		// K 9
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, K, 9);

		// D
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, D, 5);

		// R
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, tf.R, 9);

		// T
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, tf.T, 3);

		// S_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, S_rect, 2);

		// R_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, tf_rect.R, 9);

		// P_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, P_rect, 12);

		set_array(tf_rect.T, tf.T, 3);

	}
	catch(std::exception &e)
	{
		return false;
	}
	return true;
}

}

} /* namespace image_cloud */
