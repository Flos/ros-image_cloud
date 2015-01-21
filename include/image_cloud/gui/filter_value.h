/*
 * Filter_data.h
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef SRC_GUI_FILTER_DATA_H_
#define SRC_GUI_FILTER_DATA_H_

namespace image_cloud {

class Filter_value {
public:
	Filter_value();
	Filter_value(std::string name, int value, int max, bool negativ = false);
	Filter_value(std::string name, int value, int max, int numerator, int denominator, bool negativ = false, bool is_float = true);
	~Filter_value();

	void init(std::string name, int value, int max, bool negativ = false);
	void init(std::string name, int value, int max, int numerator, int denominator, bool negativ = false, bool is_float = true);
	void create_slider(std::string window_name, cv::TrackbarCallback callback, void* userdata);

	float get_value();
	bool is_initalised();
	bool negativ;
	int max;
	int numerator;
	int denominator;
	int value;
	bool initialised;
	bool is_float;
	std::string name;
};

} /* namespace image_cloud */

#endif /* SRC_GUI_FILTER_DATA_H_ */
