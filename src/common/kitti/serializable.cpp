/*
 * kitti_camera.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <image_cloud/common/kitti/serializable.h>

namespace image_cloud {

namespace kitti {

Serializable::Serializable(){
	def = ":";
	limiter = " ";
	new_line ="\n";
}

Serializable::~Serializable(){
}


/**
 * a[i] = b[i]
 */
void
Serializable::set_array(float* a, float* b, int size){
	for(int i=0; i< size; ++i){
		a[i] = b[i];
	}
}
void
Serializable::set_array(float* a, double* b, int size){
	for(int i=0; i< size; ++i){
		a[i] = b[i];
	}
}
void
Serializable::set_array(double* a, float* b, int size){
	for(int i=0; i< size; ++i){
		a[i] = b[i];
	}
}

void
Serializable::set_zero(float* a, int size){
	for(int i=0; i< size; ++i){
		a[i] = 0.0;
	}
}

void
Serializable::set_zero(double* a, int size){
	for(int i=0; i< size; ++i){
		a[i] = 0.0;
	}
}

const std::string
Serializable::get_current_date_time(){
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%d-%b-%Y %X", &tstruct);

	return buf;
}

void
Serializable::serialize_array(std::ostream &stream, float* array, int array_size){
       for(int i = 0; i < array_size; ++i){
    	   stream << limiter << array[i];
       }
       stream << new_line;
}

void
Serializable::deserialize_array(std::istream &stream, float* array, int array_size){
       for(int i = 0; i <  array_size; ++i){
    	   stream >> array[i];
       }
 }


void
Serializable::save(std::ostream &stream){
	stream << to_string();
}

bool
Serializable::save_file( std::string filename){
	std::ofstream myfile(filename.c_str());

	if (myfile.is_open()) {

		save(myfile);

		myfile.close();
	} else{
		std::cout << "Unable to open file" << def << limiter << filename << new_line;
		return false;
	}
	std::cout << "Created file" << def << limiter << filename << new_line;
	return true;
}

bool
Serializable::load_file( std::string filename){
	std::ifstream myfile(filename.c_str());

	if (myfile.is_open()) {

		load(myfile);

		myfile.close();
	} else{
		std::cout << "Unable to open file: " << filename << "\n";
		return false;
	}
	return true;


}

}

} /* namespace image_cloud */
