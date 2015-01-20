/*
 * camera_list.cpp
 *
 *  Created on: 20.01.2015
 *      Author: fnolden
 */

#include <gui/kitti/camera_list.h>

namespace image_cloud {

namespace kitti{

Camera_list::Camera_list() {
	// TODO Auto-generated constructor stub
	corner_dist = 0;
}

Camera_list::~Camera_list() {
	// TODO Auto-generated destructor stub
}

std::string
Camera_list::to_string(){
	std::stringstream ss;
	ss << "calib_time: " << get_current_date_time() << new_line;
	ss << "corner_dist: " << corner_dist << new_line;

	for(int i = 0; i < cameras.size(); ++i){
		ss << cameras.at(i).to_string();
	}

	return ss.str();
}

bool
Camera_list::load( std::istream &stream){

	std::string line;
	std::getline (stream, line); // Calib_time:

	std::cout << line << '\n';
	std::istringstream in(line);      //make a stream for the line itself

	std::string type;
	in >> type;

	// calib_time: time..
	// corner_dist: 9.950000e-02
	// S_00 ...
	if(type == "calib_time:")
	{
		std::getline (stream, line);
		in.clear();
		in.str(line);
		type.clear();

		in >> type;

		if(type == "corner_dist:"){
			in >> corner_dist;

			Camera cam;
			while( cam.load(stream) ){
				cam.set_camera_nr(cameras.size());
				cameras.push_back(cam);
			}
		}
		else{
			return false;
		}
	}
	else{
		return false;
	}
	return true;
}


}

} /* namespace image_cloud */
