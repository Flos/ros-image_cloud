/*
 * kitti_camera.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "common/kitti/tf.h"

namespace image_cloud {

namespace kitti {

Tf::Tf(){
	set_zero(R, 9);
	set_zero(T, 3);
	set_zero(delta_f, 2);
	set_zero(delta_c, 2);
	R[0] = 1;
	R[4] = 1;
	R[8] = 1;
}

void
Tf::get_transform(tf::Transform &tf){
	tf.setBasis(tf::Matrix3x3(R[0],R[1],R[2],
								R[3],R[4],R[5],
								R[6],R[7],R[8]));

	tf.setOrigin(tf::Vector3(T[0], T[1], T[2]));
}

void
Tf::get_rotation(tf::Matrix3x3 &rotation){
	rotation.setValue(R[0],R[1],R[2],
					R[3],R[4],R[5],
					R[6],R[7],R[8]);
}

void
Tf::set_transform(tf::Transform tf){
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

bool
Tf::load( std::istream &stream){
	std::string line;

	while ( std::getline (stream, line) )
	{
		std::cout << line << '\n';
		std::istringstream in(line);      //make a stream for the line itself

		std::string type;
		in >> type;                  //read the first whitespace-separated token

		//if(type == "calib_time");
		if(type == "R:")
		{
			deserialize_array(in, R, 9);
		}
		if(type == "T:")
		{
			deserialize_array(in, T, 3);
		}
		if(type == "delta_f:")
		{
			deserialize_array(in, delta_f, 2);
		}
		if(type == "delta_c:")
		{
			deserialize_array(in, delta_c, 2);
		}
	}
	std::cout << "Got: " << to_string();
	return true;

}

std::string
Tf::to_string(){
	std::stringstream ss;

	ss.setf(std::ios::scientific);

	ss << "calib_time" << def << limiter << get_current_date_time() << new_line;
	ss << "R" << def << limiter; serialize_array(ss, R, 9);
	ss << "T" << def << limiter; serialize_array(ss, T, 3);
	ss << "delta_f" << def << limiter; serialize_array(ss, delta_f, 2);
	ss << "delta_c" << def << limiter; serialize_array(ss, delta_c, 2);

	return ss.str();
}


//readTransform.setOrigin( tf::Vector3(values[3], values[7], values[11]) );
//readTransform.setBasis( tf::Matrix3x3(values[0], values[1], values[2],values[4], values[5], values[6],values[8], values[9], values[10]) );


}

} /* namespace image_cloud */
