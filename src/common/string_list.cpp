/*
 * Filelist.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <common/string_list.h>

namespace image_cloud {

String_list::String_list() {
	// TODO Auto-generated constructor stub

}

String_list::~String_list() {
	// TODO Auto-generated destructor stub
}

void
String_list::get_fullname(std::string &filepath, int index){
	if(index > list.size()){
		std::cout << "Index: " << index << " out of bounds, Max: " << list.size() << "\n";
		return;
	}
	filepath = path + list.at(index);
}

bool
String_list::load(std::istream &stream){
	std::string line;
	while ( std::getline (stream, line) )
	{
		list.push_back(line);
	}
	return true;
}

std::string
String_list::to_string(){
	std::stringstream ss;
	for(int i = 0; i < list.size(); ++i){
		ss << list.at(i) << new_line;
	}

	if(list.empty()){
		std::cout << "File is empty\n";
	}
	return ss.str();
}

} /* namespace image_cloud */
