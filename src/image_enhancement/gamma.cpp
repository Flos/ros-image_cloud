#include "image_enhancement/gamma.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Gamma, image_cloud::Gamma, nodelet::Nodelet)

namespace image_cloud {

void
Gamma::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
//	nh.param<std::string>("name", node_name_, "image_enhancement_gamma");
	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_gamma");

	it_.reset(new image_transport::ImageTransport(nh));
	sub_ = it_->subscribe(config_.subscribe_topic, 1,
			&Gamma::callback, this);
	pub_ = it_->advertise(config_.publish_topic, 1);

}

void
Gamma::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
	ROS_INFO_NAMED(node_name_,"callback");

	if(pub_.getNumSubscribers() == 0) return;

	cv_bridge::CvImagePtr cv_ptr;
	try{
	   cv_ptr = cv_bridge::toCvCopy(input_msg_image, input_msg_image->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
	   ROS_ERROR("cv_bridge exception: %s", e.what());
	   return;
	}

	cv_ptr->image = correctGamma(cv_ptr->image, config_.gamma);

	pub_.publish(cv_ptr->toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

void
Gamma::reconfigure_callback(Config &config, uint32_t level) {

	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub:\t%s", config.subscribe_topic.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t%s", config.publish_topic.c_str());
	ROS_INFO_NAMED(node_name_, "gamma: \t%f", config.gamma);

	if(config.subscribe_topic != config_.subscribe_topic){
	  sub_ = it_->subscribe(config.subscribe_topic, 1,
				&Gamma::callback, this);
	  ROS_INFO_NAMED(node_name_, "Subscribe topic changed from %s to %s", config_.subscribe_topic.c_str(), config.subscribe_topic.c_str());
	  //
	}

	if(config.publish_topic != config_.publish_topic)
	{
	  pub_ = it_->advertise(config.publish_topic, 1);
	  ROS_INFO_NAMED(node_name_, "Publish topic changed from %s to %s", config_.publish_topic.c_str(), config.publish_topic.c_str());
	}
	config_ = config;
}

Gamma::~Gamma(){
	}

} /* end namespace */
