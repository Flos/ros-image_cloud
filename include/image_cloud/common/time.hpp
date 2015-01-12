#include <ros/ros.h>
#include <std_msgs/Header.h>

#ifndef TIME_DEBUG_H_
#define TIME_DEBUG_H_

inline void
time(std::string message, const std_msgs::Header &header, bool info=false){
	ros::Time time = ros::Time::now();

	std::ostringstream stringStream;
	stringStream << message << ", Time now: " << time.sec <<"." << time.nsec << ", ";
	stringStream << "header: " << header.stamp.sec << "." << header.stamp.nsec << ", ";
	stringStream << "delta: " << (time.sec - header.stamp.sec) << "." << time.nsec - header.stamp.nsec;
	std::string copyOfStr = stringStream.str();

	if(info)
	{
		ROS_INFO("%s", copyOfStr.c_str());
	}
	else{
		ROS_DEBUG("%s", copyOfStr.c_str());
	}
}


inline void
time(std::string message, const std_msgs::Header &header,  const std_msgs::Header &header2, bool info=false){
	ros::Time time = ros::Time::now();

	std::ostringstream stringStream;
	stringStream << message << ", Time now: " << time.sec <<"." << time.nsec << ", ";
	stringStream << "header: " << header.stamp.sec << "." << header.stamp.nsec << ", ";
	stringStream << "header2: " << header2.stamp.sec << "." << header2.stamp.nsec << ", ";
	stringStream << "delta: " << (header.stamp.sec - header2.stamp.sec) << "." << header.stamp.nsec - header2.stamp.nsec;
	std::string copyOfStr = stringStream.str();

	if(info)
	{
		ROS_INFO("%s", copyOfStr.c_str());
	}
	else{
		ROS_DEBUG("%s", copyOfStr.c_str());
	}
}

#endif
