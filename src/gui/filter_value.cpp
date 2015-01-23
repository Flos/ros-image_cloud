/*
 * Filter_data.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include <gui/filter_value.h>

namespace image_cloud {

Slider::Slider() {
	value = 0;
	max = 0;
	loaded = 0;
	numerator = 0;
	denominator = 0;
	negativ = false;
	is_float = true;
	initialised = false;
}

Slider::Slider(std::string name, int value, int max, bool negativ){
	init(name, value, max, negativ);
}

Slider::Slider(std::string name, int value, int max, int numerator, int denominator, bool negativ, bool is_float) {
	init(name, value, max, numerator, denominator, negativ, is_float);
}

Slider::~Slider() {
	// TODO Auto-generated destructor stub
}

void
Slider::init(std::string name, int default_value, int max, bool negativ){
	this->name = name;
	this->value = default_value;
	this->loaded = default_value;
	this->max = max;
	this->negativ = negativ;
	this->is_float = false;
	this->initialised = true;
}

void
Slider::init(std::string name, int default_value, int max, int numerator, int denominator, bool negativ, bool is_float){
	init(name, default_value, max, negativ);

	this->numerator = numerator;
	this->denominator = denominator;
	this->is_float = is_float;
	initialised = true;
}

bool
Slider::is_initalised(){
	return initialised;
}


float
Slider::get_value(){
	float calculated_val;
	std::stringstream ss;
	ss << name << ": ";
	if(negativ){
		calculated_val = value - (max/2);
		ss << "is negativ subsctracting half: ";

	}
	else{

		calculated_val = value;
	}
	ss << "( " << calculated_val;

	if(is_float && numerator!=0 && denominator != 0){
		calculated_val = ( calculated_val * numerator) / denominator;
		ss << " * " << numerator << " ) / " << denominator;
	}
	else{
		ss << " ) ";
	}

	ss << " = " << calculated_val;
	printf("%s: %f\n%s \n", name.c_str(), calculated_val, ss.str().c_str());
	return calculated_val;
}

void Slider::create_slider(std::string window_name, cv::TrackbarCallback callback, void* userdata){
	if(max > 0){
		cv::createTrackbar( name, 	 window_name, &value, max, callback, userdata );
		if(is_float){
			cv::createTrackbar( name+" numerator",   window_name, &numerator, 		max, callback, userdata );
			cv::createTrackbar( name+" denominator", window_name, &denominator, 	max, callback, userdata );
		}
	}
}

} /* namespace image_cloud */
