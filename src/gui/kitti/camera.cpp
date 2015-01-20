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
	ss << "R_" <<  buffer; serialize_array(ss, R, 9);
	ss << "T_" <<  buffer; serialize_array(ss, T, 3);
	ss << "S_rect_" <<  buffer; serialize_array(ss, S_rect, 2);
	ss << "R_rect_" <<  buffer; serialize_array(ss, R_rect, 9);
	ss << "P_rect_" <<  buffer; serialize_array(ss, P_rect, 12);
	return ss.str();
}

void
Camera::get_camera_info(sensor_msgs::CameraInfo &info_msg, bool rect){
	info_msg.D[0] = D[0];
	info_msg.D[1] = D[1];
	info_msg.D[2] = D[2];
	info_msg.D[3] = D[3];
	info_msg.D[4] = D[4];

	info_msg.K[0] = K[0];
	info_msg.K[1] = K[1];
	info_msg.K[2] = K[2];
	info_msg.K[3] = K[3];
	info_msg.K[4] = K[4];
	info_msg.K[5] = K[5];
	info_msg.K[6] = K[6];
	info_msg.K[7] = K[7];
	info_msg.K[8] = K[8];

	info_msg.P[0] = P_rect[0];
	info_msg.P[1] = P_rect[1];
	info_msg.P[2] = P_rect[2];
	info_msg.P[3] = P_rect[3];
	info_msg.P[4] = P_rect[4];
	info_msg.P[5] = P_rect[5];
	info_msg.P[6] = P_rect[6];
	info_msg.P[7] = P_rect[7];
	info_msg.P[8] = P_rect[8];
	info_msg.P[9] = P_rect[9];
	info_msg.P[10] = P_rect[10];
	info_msg.P[11] = P_rect[11];

	if(rect){
		info_msg.R[0] = R_rect[0];
		info_msg.R[1] = R_rect[1];
		info_msg.R[2] = R_rect[2];
		info_msg.R[3] = R_rect[3];
		info_msg.R[4] = R_rect[4];
		info_msg.R[5] = R_rect[5];
		info_msg.R[6] = R_rect[6];
		info_msg.R[7] = R_rect[7];
		info_msg.R[8] = R_rect[8];
	}
	else{
		info_msg.R[0] = R[0];
		info_msg.R[1] = R[1];
		info_msg.R[2] = R[2];
		info_msg.R[3] = R[3];
		info_msg.R[4] = R[4];
		info_msg.R[5] = R[5];
		info_msg.R[6] = R[6];
		info_msg.R[7] = R[7];
		info_msg.R[8] = R[8];
	}
}

void
Camera::set_camera_info(sensor_msgs::CameraInfo info_msg, bool rect){
	D[0] = info_msg.D[0];
	D[1] = info_msg.D[1];
	D[2] = info_msg.D[2];
	D[3] = info_msg.D[3];
	D[4] = info_msg.D[4];

	K[0] = info_msg.K[0];
	K[1] = info_msg.K[1];
	K[2] = info_msg.K[2];
	K[3] = info_msg.K[3];
	K[4] = info_msg.K[4];
	K[5] = info_msg.K[5];
	K[6] = info_msg.K[6];
	K[7] = info_msg.K[7];
	K[8] = info_msg.K[8];

	P_rect[0] = info_msg.P[0];
	P_rect[1] = info_msg.P[1];
	P_rect[2] = info_msg.P[2];
	P_rect[3] = info_msg.P[3];
	P_rect[4] = info_msg.P[4];
	P_rect[5] = info_msg.P[5];
	P_rect[6] = info_msg.P[6];
	P_rect[7] = info_msg.P[7];
	P_rect[8] = info_msg.P[8];
	P_rect[9] = info_msg.P[9];
	P_rect[11] = info_msg.P[10];
	P_rect[11] = info_msg.P[11];

	R[0] = info_msg.R[0];
	R[1] = info_msg.R[1];
	R[2] = info_msg.R[2];
	R[3] = info_msg.R[3];
	R[4] = info_msg.R[4];
	R[5] = info_msg.R[5];
	R[6] = info_msg.R[6];
	R[7] = info_msg.R[7];
	R[8] = info_msg.R[8];
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
		deserialize_array(in, R, 9);

		// T
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, T, 3);

		// S_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, S_rect, 2);

		// R_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, R_rect, 9);

		// P_rect
		std::getline (stream, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, P_rect, 12);
	}
	catch(std::exception &e)
	{
		return false;
	}
	return true;
}

}

} /* namespace image_cloud */
