/*
 * config_parser.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "gui/config_parser.h"

namespace image_cloud {

Config_parser::Config_parser() {
	// TODO Auto-generated constructor stub
	def = ":";
	limiter = " ";
	new_line ="\n";
}

Config_parser::~Config_parser() {
	// TODO Auto-generated destructor stub
}

void Config_parser::write_camera_info() {

	float val = 2;
	int nr = 1;
	char buffer[2];
	sprintf(buffer,"%02d", nr);

	std::ofstream myfile("example.txt");
	if (myfile.is_open()) {
		myfile.setf(std::ios::fixed, std::ios::floatfield);
		myfile << "K_" <<  buffer << def <<limiter << val << new_line;
		myfile << "This is another line.\n";
		myfile.close();
	} else
		std::cout << "Unable to open file";
}


/**
* calib_time: 15-Mar-2012 11:37:16
* R: 7.533745e-03 -9.999714e-01 -6.166020e-04 1.480249e-02 7.280733e-04 -9.998902e-01 9.998621e-01 7.523790e-03 1.480755e-02
* T: -4.069766e-03 -7.631618e-02 -2.717806e-01
* delta_f: 0.000000e+00 0.000000e+00
* delta_c: 0.000000e+00 0.000000e+00
*/
bool
Config_parser::write_calib_velo_to_cam(std::string filename, kitti::velo_to_cam tf ) {
	std::ofstream myfile(filename.c_str());
	if (myfile.is_open()) {
		myfile.setf(std::ios::scientific);
		myfile << "calib_time" << def << limiter << get_current_date_time() << new_line;
		myfile << "R" << def << limiter; write_array(myfile, tf.R, 9);
		myfile << "T" << def << limiter; write_array(myfile, tf.T, 3);
		myfile << "delta_f" << def << limiter; write_array(myfile, tf.delta_f, 2);
		myfile << "delta_c" << def << limiter; write_array(myfile, tf.delta_c, 2);
		std::cout << "Created file: " << filename << new_line << new_line;
		myfile.close();
	} else{
		std::cout << "Unable to open file";
		return false;
	}
	return true;
}

bool
Config_parser::load_calib_velo_to_cam(std::string filename, kitti::velo_to_cam &tf ) {
	std::string line;
	std::ifstream myfile(filename.c_str());

	std::cout << "Loading calib_velo_to_cam from file:" << filename << new_line;

	if (myfile.is_open()){
		while ( std::getline (myfile, line) )
		{
			std::cout << line << '\n';
			std::istringstream in(line);      //make a stream for the line itself

			std::string type;
			in >> type;                  //read the first whitespace-separated token

			//if(type == "calib_time");
			if(type == "R:")
			{
				read_array(in, tf.R, 9);
			}
			if(type == "T:")
			{
				read_array(in, tf.T, 3);
			}
			if(type == "delta_f:")
			{
				read_array(in, tf.delta_f, 2);
			}
			if(type == "delta_c:")
			{
				read_array(in, tf.delta_c, 2);
			}
		}

		myfile.close();
	}else{
		std::cout << "Unable to open file";
		return false;
	}
	return true;
}

bool
Config_parser::load_calib_cam_to_cam(std::string filename, float &corner_distance, std::vector<kitti::camera> &cams){
	assert( cams.empty() );
	assert( !filename.empty() );

	std::string line;
	std::ifstream myfile(filename.c_str());

	std::cout << "Loading calib_cam_to_cam from file:" << filename << new_line;

	if (myfile.is_open()){
		while ( std::getline (myfile, line) )
		{
			std::cout << line << '\n';
			std::istringstream in(line);      //make a stream for the line itself

			std::string type;
			in >> type;                  //read the first whitespace-separated token

			if(type == "corner_dist:")
			{
				in >> corner_distance;
				kitti::camera cam;
				while(cam.load(myfile)){;
					cams.push_back(cam);
				}
			}
		}

		myfile.close();
	}else{
		std::cout << "Unable to open file";
		return false;
	}

	return true;
}

bool
Config_parser::write_calib_cam_to_cam(std::string filename, float corner_distance, std::vector<kitti::camera> &cams){

	std::ofstream myfile(filename.c_str());
	if (myfile.is_open()) {
		myfile.setf(std::ios::scientific);
		myfile << "calib_time"  << def << limiter << get_current_date_time() << new_line;
		myfile << "corner_dist" << def << limiter << corner_distance << new_line;

		for(int i = 0; i < cams.size(); ++i){
			myfile << cams.at(i).save(i);
		}

		myfile.close();
	} else{
		std::cout << "Unable to open file";
		return false;
	}
	return true;
}

void Config_parser::write_array(std::ofstream &stream, float* array, int size){
	for(int i = 0; i < size; ++i){
		stream << array[i] << limiter;
	}
	stream << new_line;
}

void
Config_parser::read_array(std::istringstream &in, float* array, int size){
	for(int i = 0; i < size; ++i){
		in >> array[i];       //now read the whitespace-separated floats
	}
}

// Get current date/time, format is DD-MON-YYYY.HH:mm:ss
const std::string
Config_parser::get_current_date_time() {
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
Config_parser::read_camera(){

}
} /* namespace image_cloud */
