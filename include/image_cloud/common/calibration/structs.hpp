/*
 * structs.h
 *
 *  Created on: 28.01.2015
 *      Author: fnolden
 */

#include <assert.h>

#ifndef INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_
#define INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_

namespace search
{

struct Value{
	float min;
	float max;
	int steps_max;
	float step_width;

	Value(){
		init_range(0, 0, 1);
	};

	Value( float value, float range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
	}

	void init_range( float value, float range, int steps_max = 3){
		init_min_max(value - range/2, value + range/2, steps_max);
	}

	void init_min_max(float min, float max, int steps_max = 3){
		assert(min <= max);
		assert(steps_max > 0);
		this->min = min;
		this->max = max;
		this->steps_max = steps_max;
		if(steps_max != 1){
			step_width = (max - min)/(steps_max-1); // First step = min, last step = max
		}
		else{
			step_width = 0;
		}
	}

	float at(int step){
		assert(step < steps_max);
		return min + (step_width*step);
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "min: " << min << " max: " << max << " steps: " << steps_max << " step_width: " << step_width << "\n";
		return ss.str();
	}
};

struct Search_setup{
	Value x;
	Value y;
	Value z;
	Value roll;
	Value pitch;
	Value yaw;
	std::string to_string(){
		std::stringstream ss;
		ss << "x: " << x.to_string();
		ss << "y: " << y.to_string();
		ss << "z: " << z.to_string();
		ss << "roll: " << roll.to_string();
		ss << "pitch: " << pitch.to_string();
		ss << "yaw: " << yaw.to_string();
		return ss.str();
	}
};

struct Search_value{
	Search_value(){
		init(0, 0, 0, 0, 0, 0, 0);
	}

	Search_value(tf::Transform tf, long unsigned int result = 0){
			init(tf, result);
		}

	Search_value( float x, float y, float z, float roll, float pitch, float yaw, long unsigned int result = 0){
		init(x, y, z, roll, pitch, yaw, result);
	}

	void init(float x, float y, float z, float roll, float pitch, float yaw, long unsigned int result = 0)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->roll = roll;
		this->pitch = pitch;
		this->yaw = yaw;
		this->result = result;
	}

	void init(tf::Transform tf, long unsigned int result = 0)
		{

			this->x = tf.getOrigin()[0];
			this->y = tf.getOrigin()[1];
			this->z = tf.getOrigin()[2];
			double r,p,y;
			tf.getBasis().getRPY(r, p, y);
			this->roll = r;
			this->pitch = p;
			this->yaw = y;
			this->result = result;
		}

	std::string to_string(){
		std::stringstream ss;
		ss << "x: " << x << " y: " << y <<" z: " << z;
		ss << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
		ss << " result: " << result << "\n";
		return ss.str();
	}

	void get_transform(tf::Transform &tf){
		tf.setOrigin( tf::Vector3( x, y, z ) );

		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw );
		tf.setRotation( q );
	}

	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	long unsigned int result;
};

}
#endif /* INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_ */
