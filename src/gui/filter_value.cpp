/*
 * Filter_data.cpp
 *
 *  Created on: 19.01.2015
 *      Author: fnolden
 */

#include "gui/filter_value.h"

namespace image_cloud {

Filter_value::Filter_value() {
	value = 0;
	max = 0;
	numerator = 0;
	denominator = 0;
	negativ = false;
	is_float = true;
	initialised = false;
}

Filter_value::Filter_value(std::string name, int value, int max, int numerator, int denominator, bool negativ, bool is_float) {
	init(name, value, max, numerator, denominator, negativ, is_float);
}

Filter_value::~Filter_value() {
	// TODO Auto-generated destructor stub
}

void
Filter_value::init(std::string name, int value, int max, bool negativ){
	this->name = name;
	this->value = 0;
	this->max = max;
	this->negativ = negativ;
	this->is_float = false;
	this->initialised = true;
}

void
Filter_value::init(std::string name, int value, int max, int numerator, int denominator, bool negativ, bool is_float){
	this->value = value;
	this->max = max;
	this->numerator = numerator;
	this->denominator = denominator;
	this->negativ = negativ;
	this->name = name;
	this->is_float = is_float;
	initialised = true;
}

bool
Filter_value::is_initalised(){
	return initialised;
}

float
Filter_value::get_value(){
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

} /* namespace image_cloud */
