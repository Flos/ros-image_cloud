/*
 * Edgedetectornodelet.cpp
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#include "image_enhancement/erode.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Erode, image_cloud::Erode, nodelet::Nodelet)

namespace image_cloud {

void
Erode::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_features_edge");

	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_erode");

	it_.reset(new image_transport::ImageTransport(nh));
	sub_ = it_->subscribe(config_.subscribe_topic, 1,
			&Erode::callback, this);
	pub_ = it_->advertise(config_.publish_topic, 1);
//
	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Erode::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Erode::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	cv::Mat dst;

	try{
		cv::erode( cv_ptr->image, dst, element);

	}catch (cv::Exception &e){
		ROS_ERROR_NAMED(node_name_,"cv_bridge exception: %s", e.what());
		return;
	}

	cv_bridge::CvImage image_edge;

	image_edge = cv_bridge::CvImage(cv_ptr->header,  input_msg_image->encoding, dst);


	pub_.publish(image_edge.toImageMsg());
	ROS_INFO_NAMED(node_name_,"callback end");
}

void
Erode::reconfigure_callback(Config &config, uint32_t level) {
  ROS_INFO_NAMED(node_name_, "Reconfigure Request");

  ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
  ROS_INFO_NAMED(node_name_, "subscribe_topic:\t%s", config.subscribe_topic.c_str());
  ROS_INFO_NAMED(node_name_, "publish_topic:\t%s", config.publish_topic.c_str());
  ROS_INFO_NAMED(node_name_, "erosion_size: \t%i", config.erosion_size);
  ROS_INFO_NAMED(node_name_, "element_form: \t%i", config.element_form);

  if(config.subscribe_topic != config_.subscribe_topic
 	  || config.publish_topic != config_.publish_topic)
  {
	  ROS_ERROR_NAMED(node_name_, "restarting of node needed");
  }

  int erosion_type = 0;
  switch(config.element_form){
	  default:
	  case 0:
		  erosion_type = cv::MORPH_RECT;
		  break;
	  case 1:
		  erosion_type = cv::MORPH_CROSS;
		  break;
	  case 2:
		  erosion_type = cv::MORPH_ELLIPSE;
		  break;
  }

  element = cv::getStructuringElement( erosion_type,
  										   cv::Size( 2*config.erosion_size + 1, 2*config.erosion_size+1 ),
  										   cv::Point( config.erosion_size, config.erosion_size )
  										   );

  config_ = config;
}

Erode::Erode() {
	// TODO Auto-generated constructor stub
}

Erode::~Erode() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
