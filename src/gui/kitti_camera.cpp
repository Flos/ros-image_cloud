/*
 * kitti_camera.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "gui/kitti_camera.h"

namespace image_cloud {


/**
 * a[i] = b[i]
 */
void set_array(double *a, float*b, int size){
	for(int i=0; i< size; ++i){
		a[i] = b[i];
	}
}

void set_array(float *a, double*b, int size){
	for(int i=0; i< size; ++i){
		a[i] = b[i];
	}
}

std::string
kitti::camera::save(int camer_nr){
	std::stringstream ss;

	char buffer[3];
	sprintf(buffer,"%02d:", camer_nr);

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
kitti::camera::get_camera_info(sensor_msgs::CameraInfo &info_msg, bool rect){
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
kitti::camera::set_camera_info(sensor_msgs::CameraInfo info_msg, bool rect){
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

void
kitti::camera::serialize_array(std::stringstream &ss, float* array, int array_size){
	for(int i = 0; i < array_size; ++i){
			ss << " "<< array[i];
	}
	ss << "\n";
}

void
kitti::camera::deserialize_array(std::istringstream &in, float* array, int array_size){
	for(int i = 0; i <  array_size; ++i){
			in >> array[i];
	}
}

bool
kitti::camera::load(std::ifstream &file){
	std::string line;
	std::istringstream in;
	std::string type;

	try{
		// S 2
		if(std::getline (file, line) <= 0) return false;
		in.str(line);
		in >> type;
		deserialize_array(in, S, 2);

		// K 9
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, K, 9);

		// D
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, D, 5);

		// R
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, R, 9);

		// T
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, T, 3);

		// S_rect
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, S_rect, 2);

		// R_rect
		std::getline (file, line);
		in.clear(); in.str(line);
		in >> type;
		deserialize_array(in, R_rect, 9);

		// P_rect
		std::getline (file, line);
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

void
kitti::velo_to_cam::get_transform(tf::Transform &tf){
	tf.setBasis(tf::Matrix3x3(R[0],R[1],R[2],
								R[3],R[4],R[5],
								R[6],R[7],R[8]));

	tf.setOrigin(tf::Vector3(T[0], T[1], T[2]));
}

void
kitti::velo_to_cam::set_transform(tf::Transform tf){
	tf::Matrix3x3 mat = tf.getBasis();
	R[0] = mat[0][0];
	R[1] = mat[0][1];
	R[2] = mat[0][2];
	R[3] = mat[1][0];
	R[4] = mat[1][1];
	R[5] = mat[1][2];
	R[6] = mat[2][0];
	R[7] = mat[2][1];
	R[8] = mat[2][2];

	T[0] = tf.getOrigin()[0];
	T[1] = tf.getOrigin()[1];
	T[2] = tf.getOrigin()[2];
}

} /* namespace image_cloud */
