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
	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_edge");

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
	NODELET_DEBUG("callback");

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

	NODELET_DEBUG("callback end");
}

void
Edge_detector_nodelet::reconfigure_callback(Config &config, uint32_t level) {
  NODELET_INFO( "Reconfigure Request");

  NODELET_INFO( "name:\t%s", node_name_.c_str());
  NODELET_INFO( "subscribe_topic:\t%s", config.subscribe_topic.c_str());
  NODELET_INFO( "publish_topic:\t%s", config.publish_topic.c_str());
  NODELET_INFO( "kernel_size: \t%i", config.kernel_size);
  NODELET_INFO( "filter: \t%i", config.filter);
  NODELET_INFO( "threshold1: \t%f", config.threshold1);
  NODELET_INFO( "threshold2: \t%f", config.threshold2);
  NODELET_INFO( "publish_color: \t%s", config.publish_color ? "true" : "false");

  if(config.subscribe_topic != config_.subscribe_topic){
  	  sub_ = it_->subscribe(config.subscribe_topic, 1,
  	  			&Edge_detector_nodelet::callback, this);
  	  NODELET_INFO( "Subscribe topic changed from %s to %s", config_.subscribe_topic.c_str(), config.subscribe_topic.c_str());
  	  //
  }

  if(config.publish_topic != config_.publish_topic)
  {
  	  pub_ = it_->advertise(config.publish_topic, 1);
  	  NODELET_INFO( "Publish topic changed from %s to %s", config_.publish_topic.c_str(), config.publish_topic.c_str());
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
