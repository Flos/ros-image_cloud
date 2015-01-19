/*
 * Filelist.h
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
#include <vector>

#ifndef SRC_GUI_FILELIST_H_
#define SRC_GUI_FILELIST_H_

namespace image_cloud {

class Filelist {
public:
	Filelist();
	virtual ~Filelist();
	std::string path;
	std::vector<std::string> file_names;
	void get_fullname(std::string &filepath, int index);
	bool load(std::string filename);
};

} /* namespace image_cloud */

#endif /* SRC_GUI_FILELIST_H_ */
