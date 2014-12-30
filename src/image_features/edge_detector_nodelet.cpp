/*
 * Edgedetectornodelet.cpp
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#include "edge_detector_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Edge_detector_nodelet, image_cloud::Edge_detector_nodelet, nodelet::Nodelet)

namespace image_cloud {

void
Edge_detector_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_features_edge");
	nh.param<std::string>("sub", subscribe_topic_, "");
	nh.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_edge");
	nh.param<int>("kernel", kernel_size_, 5);
	nh.param<int>("filter", filter_, 0);
	nh.param<int>("threshold1", threshold1_, 50);
	nh.param<int>("threshold2", threshold2_, 200);
	nh.param<bool>("color", publish_color_ , false);


	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub:\t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "kernel: \t%i", kernel_size_);
	ROS_INFO_NAMED(node_name_, "filter: \t%i", filter_);
	ROS_INFO_NAMED(node_name_, "threshold1: \t%i", threshold1_);
	ROS_INFO_NAMED(node_name_, "threshold2: \t%i", threshold2_);
	ROS_INFO_NAMED(node_name_, "color: \t%i", publish_color_);

	if(subscribe_topic_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}

	it_ = new image_transport::ImageTransport(nh);
	sub_ = it_->subscribe(subscribe_topic_, 1, &Edge_detector_nodelet::callback, this);
	pub_ = it_->advertise(publish_topic_, 1);
}

void
Edge_detector_nodelet::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
	ROS_INFO_NAMED(node_name_,"callback");

	if(pub_.getNumSubscribers() == 0) return;

	cv_bridge::CvImagePtr cv_ptr;
	try{
	   cv_ptr = cv_bridge::toCvCopy(input_msg_image, input_msg_image->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
	   ROS_ERROR_NAMED(node_name_, "cv_bridge exception: %s", e.what());
	   return;
	}

	cv::Mat src_gray, dst_gray, dst_color;

	cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

	switch(filter_){
		case 0:
			Canny( src_gray, dst_gray, threshold1_, threshold2_, kernel_size_ );
			//Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

			break;
	    case 1:
			//cv::Laplacian( cv_ptr->image, image_blur.image, cv::Size( kernel_size_, kernel_size_ ), cv::Point(-1,-1) );
			break;
	    default :
	    	ROS_ERROR_NAMED(node_name_, "Filter not implemented, select filter between 0 and 3:");
	}

	cv_bridge::CvImage image_edge;

	if(publish_color_){
		 cvtColor(dst_gray, dst_color, CV_GRAY2BGR);
		 image_edge = cv_bridge::CvImage(cv_ptr->header, input_msg_image->encoding, dst_color);
	}
	else{
		image_edge = cv_bridge::CvImage(cv_ptr->header, sensor_msgs::image_encodings::MONO8, dst_gray);
	}

	pub_.publish(image_edge.toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

Edge_detector_nodelet::Edge_detector_nodelet() {
	// TODO Auto-generated constructor stub
	it_ = 0;
	kernel_size_ = 0;
	filter_ = 0;
	threshold1_ = 0;
	threshold2_ = 0;
	publish_color_ = 0;
}

Edge_detector_nodelet::~Edge_detector_nodelet() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
