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

#ifndef INCLUDE_SERIALIZABLE_H_
#define INCLUDE_SERIALIZABLE_H_

namespace image_cloud {

namespace kitti{

class Serializable{
public:
	Serializable();
	virtual ~Serializable();

	virtual void save( std::ostream &stream);
	virtual bool save_file( std::string filename);
	virtual bool load_file( std::string filename);

	virtual const std::string get_current_date_time();

	virtual std::string to_string() = 0;
	virtual bool load( std::istream &stream) = 0;

protected:
	void serialize_array(std::ostream &stream, float*, int array_size);
	void deserialize_array( std::istream &stream, float*, int array_size);

	template <class T1, class T2>
	void set_array(T1* a, T2* b, int size);

	std::string def;
	std::string limiter;
	std::string new_line;
};

}

} /* namespace image_cloud */

#endif /* INCLUDE_SERIALIZABLE_H_ */
