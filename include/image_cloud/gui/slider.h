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

class Slider {
public:
	Slider();
	Slider(std::string name, int value, int max, bool negativ = false);
	Slider(std::string name, int value, int max, int numerator, int denominator, bool negativ = false, bool is_float = true);
	~Slider();

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
	int loaded;
	bool initialised;
	bool is_float;
	std::string name;
};

} /* namespace image_cloud */

#endif /* SRC_GUI_FILTER_DATA_H_ */
