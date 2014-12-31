#include "image_enhancement/brightness.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Brightness, image_cloud::Brightness, nodelet::Nodelet)

namespace image_cloud {

void
Brightness::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_enhancement_brightness");
	nh.param<std::string>("sub", subscribe_topic_, "");
	nh.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_brightness");
	nh.param<double>("alpha", alpha_, 4);
	nh.param<double>("beta", beta_, 0);

	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub:\t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "alpha: \t%f", alpha_);
	ROS_INFO_NAMED(node_name_, "beta: \t%f", beta_);

	if(subscribe_topic_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}

	it_ = new image_transport::ImageTransport(nh);
	sub_ = it_->subscribe(subscribe_topic_, 1, &Brightness::callback, this);
	pub_ = it_->advertise(publish_topic_, 1);
}

void
Brightness::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	//Brightness
	cv_ptr->image = brightness(cv_ptr->image, alpha_, beta_);

	pub_.publish(cv_ptr->toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

Brightness::~Brightness(){
		delete it_;
	}

} /* end namespace */
