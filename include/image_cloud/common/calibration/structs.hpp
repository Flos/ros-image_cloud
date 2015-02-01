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
const std::string spacer = " \t";

struct Value_calculator{
	float min;
	float max;
	int steps_max;
	float step_width;

	Value_calculator(){
		init_range(0, 0, 1);
	};

	Value_calculator( float value, float range, int steps_max = 3){
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
		ss << "min:" << spacer << min << spacer;
		ss << "max:" << spacer << max << spacer;
		ss << "steps:" << spacer << steps_max << spacer;
		ss << "step_width:"<< spacer << step_width << spacer;
		return ss.str();
	}
};

struct Search_setup{
	Value_calculator x;
	Value_calculator y;
	Value_calculator z;
	Value_calculator roll;
	Value_calculator pitch;
	Value_calculator yaw;

	Search_setup(){
		init(0,0,0,0,0,0,0,1);
	}

	Search_setup(float x, float y, float z, float roll, float pitch, float yaw, float range, int steps){
			init( x, y, z, roll, pitch, yaw, range, steps);
		}

	void init(float x, float y, float z, float roll, float pitch, float yaw, float range, int steps){
		this->x.init_range(x, range, steps);
		this->y.init_range(y, range, steps);
		this->z.init_range(z, range, steps);
		this->roll.init_range(roll, range, steps);
		this->pitch.init_range(pitch, range, steps);
		this->yaw.init_range(yaw, range, steps);
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "x:" << spacer << x.to_string() << spacer;
		ss << "y:" << spacer << y.to_string() << spacer;
		ss << "z:" << spacer << z.to_string() << spacer;
		ss << "roll:" << spacer << roll.to_string() << spacer;
		ss << "pitch:" << spacer << pitch.to_string() << spacer;
		ss << "yaw:" << spacer << yaw.to_string() << spacer;
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
		ss << "x:" << spacer << x << spacer;
		ss << "y:" << spacer << y << spacer;
		ss <<" z:" << spacer << z << spacer;
		ss << "roll:" << spacer << roll << spacer;
		ss << "pitch:" << spacer << pitch << spacer;
		ss << "yaw:" << spacer << yaw << spacer;
		ss << "result:" << spacer << result;
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

struct Multi_search_result{
	Search_value in;
	Search_value best;
	long unsigned int nr_total;
	long unsigned int nr_worse;

	Multi_search_result(){
		init();
	}

	void init(Search_value in, Search_value best, long unsigned int nr_total, long unsigned int nr_worse){
		this->nr_total = nr_total;
		this->nr_worse = nr_worse;
		this->in = in;
		this->best = best;
	}

	void init(){
		nr_total = 0;
		nr_worse = 0;
		Search_value empty;
		empty.init(0,0,0,0,0,0,0);
		in = empty;
		best = empty;
	}

	float get_fc(){
		if(nr_total > 0){
			return (float)nr_worse/(float)nr_total;
		}
		return 0;
	}

	Search_value get_delta_best(){
		Search_value delta = in;
		delta.x-=best.x;
		delta.y-=best.y;
		delta.z-=best.z;
		delta.roll-=best.roll;
		delta.pitch-=best.pitch;
		delta.yaw-=best.yaw;
		delta.result-=best.result;
		return delta;
	}

	std::string to_string(){
		std::stringstream ss;
		ss << "total:" << spacer << nr_total << spacer;
		ss << "worse:" << spacer << nr_worse << spacer;
		ss << "fc:" << spacer << get_fc() << spacer;
		ss << "in:" <<spacer << in.to_string() << spacer;
		ss << "out:" << spacer << best.to_string() << spacer;
		ss << "delta:" << spacer << get_delta_best().to_string() << spacer;

		return ss.str();
	}
};

struct Window{
	int window_size;
	std::deque<cv::Mat> images;
	std::deque<pcl::PointCloud<pcl::PointXYZI> > pointclouds;

	void check(){
		assert(images.size() == pointclouds.size());

		if(size() > window_size){
			pop_front();
		}
	}

	size_t size(){
		check();
		return images.size();
	}

	void pop_front(){
		images.pop_front();
		pointclouds.pop_front();

		check();
	}

	void push_back(cv::Mat image, pcl::PointCloud<pcl::PointXYZI> pointcloud){
		images.push_back(image);
		pointclouds.push_back(pointcloud);

		check();
	}
};

}
#endif /* INCLUDE_IMAGE_CLOUD_COMMON_CALIBRATION_STRUCTS_H_ */
