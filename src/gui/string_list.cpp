/*
 * Filelist.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "gui/string_list.h"

namespace image_cloud {

String_list::String_list() {
	// TODO Auto-generated constructor stub

}

String_list::~String_list() {
	// TODO Auto-generated destructor stub
}

void
String_list::get_fullname(std::string &filepath, int index){
	if(index > file_names.size()){
		std::cout << "Index: " << index << " out of bounds, Max: " << file_names.size() << "\n";
		return;
	}
	filepath = path + file_names.at(index);
}

bool
String_list::load(std::string filename){
	std::string line;
	std::ifstream myfile(filename.c_str());

	std::cout << "Loading filenames from file:" << filename << "\n";

	if (myfile.is_open()){
		while ( std::getline (myfile, line) )
		{
			file_names.push_back(line);
		}
		myfile.close();
	}else{
		std::cout << "Unable to open file: " << filename.c_str() << "\n";
		return false;
	}
	if(file_names.empty()){
		std::cout << "File: "<< filename.c_str() <<" is empty \n";
		return false;
	}
	return true;
}

} /* namespace image_cloud */
