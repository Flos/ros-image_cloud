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

void Edge_detector_nodelet::reset_image_transport() {
//	try{	sub_.shutdown(); 	}catch(std::exception &e){}
//	try{	pub_.shutdown(); 	}catch(std::exception &e){}
}

void
Edge_detector_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_features_edge");

//	//ist this needed?
	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_edge");
//	nh.param<int>("kernel", config_.kernel_size, 5);
//	nh.param<int>("filter", config_.filter, 0);
//	nh.param<int>("threshold1", config_.threshold1, 50);
//	nh.param<int>("threshold2", config_.threshold2, 200);
//	nh.param<bool>("color", config_.publish_color, false);
//
//	// 2. Info
//	if(config_.subscribe_topic.empty()) {
//		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
//	}

	it_.reset(new image_transport::ImageTransport(nh));
	sub_ = it_->subscribe(config_.subscribe_topic, 1,
			&Edge_detector_nodelet::callback, this);
	pub_ = it_->advertise(config_.publish_topic, 1);
//
	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Edge_detector_nodelet::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
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

	if(input_msg_image->encoding == sensor_msgs::image_encodings::MONO8){
		src_gray = cv_ptr->image;
	}
	else{
		cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );
	}

	try{
		switch(config_.filter){
			case 0:
				cv::Canny( src_gray, dst_gray, config_.threshold1, config_.threshold2, config_.kernel_size, config_.L2gradient );
				break;
			case 1:
				cv::Laplacian( src_gray, dst_gray, CV_16S, config_.kernel_size, 1 , 0 );
				break;
			default :
				ROS_ERROR_NAMED(node_name_, "Filter not implemented, select filter between 0 and 3:");
		}
	}catch (cv::Exception &e){
		ROS_ERROR_NAMED(node_name_,"cv_bridge exception: %s", e.what());
	}

	cv_bridge::CvImage image_edge;

	if(config_.publish_color){
		 cvtColor(dst_gray, dst_color, CV_GRAY2BGR);
		 image_edge = cv_bridge::CvImage(cv_ptr->header, input_msg_image->encoding, dst_color);
	}
	else{
		image_edge = cv_bridge::CvImage(cv_ptr->header, sensor_msgs::image_encodings::MONO8, dst_gray);
	}

	pub_.publish(image_edge.toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

void
Edge_detector_nodelet::reconfigure_callback(Config &config, uint32_t level) {
  ROS_INFO_NAMED(node_name_, "Reconfigure Request");

  ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
  ROS_INFO_NAMED(node_name_, "subscribe_topic:\t%s", config.subscribe_topic.c_str());
  ROS_INFO_NAMED(node_name_, "publish_topic:\t%s", config.publish_topic.c_str());
  ROS_INFO_NAMED(node_name_, "kernel_size: \t%i", config.kernel_size);
  ROS_INFO_NAMED(node_name_, "filter: \t%i", config.filter);
  ROS_INFO_NAMED(node_name_, "threshold1: \t%f", config.threshold1);
  ROS_INFO_NAMED(node_name_, "threshold2: \t%f", config.threshold2);
  ROS_INFO_NAMED(node_name_, "publish_color: \t%s", config.publish_color ? "true" : "false");

  if(config.subscribe_topic != config_.subscribe_topic
 	  || config.publish_topic != config_.publish_topic)
  {
	  ROS_INFO_NAMED(node_name_, "restarting image transport");
 	  reset_image_transport();
  }
  config_ = config;
}

Edge_detector_nodelet::Edge_detector_nodelet() {
	// TODO Auto-generated constructor stub
}

Edge_detector_nodelet::~Edge_detector_nodelet() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
