/*
 * Edgedetectornodelet.cpp
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#include "image_enhancement/dilate.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Dilate, image_cloud::Dilate, nodelet::Nodelet)

namespace image_cloud {

void
Dilate::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_features_edge");

	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_Dilate");

	it_.reset(new image_transport::ImageTransport(nh));
	sub_ = it_->subscribe(config_.subscribe_topic, 1,
			&Dilate::callback, this);
	pub_ = it_->advertise(config_.publish_topic, 1);
//
	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Dilate::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Dilate::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	cv::Mat dst;

	try{
		cv::dilate( cv_ptr->image, dst, element);

	}catch (cv::Exception &e){
		ROS_ERROR_NAMED(node_name_,"cv_bridge exception: %s", e.what());
		return;
	}

	cv_bridge::CvImage image_edge;

	image_edge = cv_bridge::CvImage(cv_ptr->header,  input_msg_image->encoding, dst);


	pub_.publish(image_edge.toImageMsg());
	NODELET_DEBUG("callback end");
}

void
Dilate::reconfigure_callback(Config &config, uint32_t level) {
  NODELET_INFO( "Reconfigure Request");

  NODELET_INFO( "name:\t%s", node_name_.c_str());
  NODELET_INFO( "subscribe_topic:\t%s", config.subscribe_topic.c_str());
  NODELET_INFO( "publish_topic:\t%s", config.publish_topic.c_str());
  NODELET_INFO( "dilation_size: \t%i", config.dilation_size);
  NODELET_INFO( "element_form: \t%i", config.element_form);

  if(config.subscribe_topic != config_.subscribe_topic){
	  sub_ = it_->subscribe(config.subscribe_topic, 1,
	  			&Dilate::callback, this);
	  NODELET_INFO( "Subscribe topic changed from %s to %s", config_.subscribe_topic.c_str(), config.subscribe_topic.c_str());
	  //
  }

  if(config.publish_topic != config_.publish_topic)
  {
	  pub_ = it_->advertise(config.publish_topic, 1);
	  NODELET_INFO( "Publish topic changed from %s to %s", config_.publish_topic.c_str(), config.publish_topic.c_str());
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
  										   cv::Size( 2*config.dilation_size + 1, 2*config.dilation_size+1 ),
  										   cv::Point( config.dilation_size, config.dilation_size )
  										   );

  config_ = config;
}

Dilate::Dilate() {
	// TODO Auto-generated constructor stub
}

Dilate::~Dilate() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
