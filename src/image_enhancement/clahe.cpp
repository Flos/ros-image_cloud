#include "image_enhancement/clahe.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Clahe, image_cloud::Clahe, nodelet::Nodelet)

namespace image_cloud {

void
Clahe::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_enhancement_clahe");
	nh.param<std::string>("sub", subscribe_topic_, "");
	nh.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_clahe");
	nh.param<double>("cliplimit", cliplimit_, 4);

	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub:\t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "cliplimit: \t%f", cliplimit_);

	if(subscribe_topic_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}

	it_ = new image_transport::ImageTransport(nh);
	sub_ = it_->subscribe(subscribe_topic_, 1, &Clahe::callback, this);
	pub_ = it_->advertise(publish_topic_, 1);
}


void
Clahe::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	cv_ptr->image = clahe(cv_ptr->image, cliplimit_);

	pub_.publish(cv_ptr->toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}


Clahe::~Clahe(){
		delete it_;
	}

} /* end namespace */
