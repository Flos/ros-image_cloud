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
	int step_count;
	float step_width;

	Value( float value, float range, int step_count = 3){
		init_min_max(value - range/2, value + range/2, step_count);
	}

	void init_range( float value, float range, int step_count = 3){
		init_min_max(value - range/2, value + range/2, step_count);
	}

	void init_min_max(float min, float max, int step_count = 3){
		assert(min < max);
		assert(step_count > 1);
		this->min = min;
		this->max = max;
		this->step_count = step_count;
		step_width = (max - min)/step_count;
	}

	float at(int step){
		assert(step < step_count);
		return min + (step_width*step);
	}
};

struct Search_setup{
	Value x;
	Value y;
	Value z;
	Value roll;
	Value pitch;
	Value yaw;
};

struct Search_value{
	Search_value(){
		init(0, 0, 0, 0, 0, 0, 0);
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

	std::string to_string(){
		std::stringstream ss;
		ss << "x: " << x << " y: " << y <<" z: " << z;
		ss << " roll: " << roll << " pitch: " << pitch << " yaw: " << yaw;
		ss << "result: " << result << "\n";
		return ss.str();
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
