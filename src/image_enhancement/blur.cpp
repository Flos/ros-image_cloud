#include "image_enhancement/blur.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Blur, image_cloud::Blur, nodelet::Nodelet)

namespace image_cloud {

void
Blur::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_enhancement_blur");
	nh.param<std::string>("sub", subscribe_topic_, "");
	nh.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_blur");
	nh.param<int>("kernel", kernel_size_, 5);
	nh.param<int>("filter", filter_, 0);


	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub:\t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "kernel: \t%i", kernel_size_);
	ROS_INFO_NAMED(node_name_, "filter: \t%i", filter_);

	if(subscribe_topic_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}

	it_ = new image_transport::ImageTransport(nh);
	sub_ = it_->subscribe(subscribe_topic_, 1, &Blur::callback, this);
	pub_ = it_->advertise(publish_topic_, 1);
}

void
Blur::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	cv::Mat image;

	image = cv::Mat::zeros(cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.type());
	cv_bridge::CvImage image_blur(cv_ptr->header, input_msg_image->encoding, image);


	switch(filter_){
		case 0:
			cv::bilateralFilter ( cv_ptr->image, image_blur.image, kernel_size_, kernel_size_*2, kernel_size_/2 );
			break;
	    case 1:
			cv::blur( cv_ptr->image, image_blur.image, cv::Size( kernel_size_, kernel_size_ ), cv::Point(-1,-1) );
			break;
	    case 2:
			cv::GaussianBlur( cv_ptr->image, image_blur.image, cv::Size( kernel_size_, kernel_size_ ), 0, 0 );
			break;
	    case 3:
	    	cv::medianBlur ( cv_ptr->image, image_blur.image, kernel_size_ );
	    	break;
	    default :
	    	ROS_ERROR_NAMED(node_name_, "Filter not implemented, select filter between 0 and 3:");
	}

	pub_.publish(image_blur.toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

Blur::~Blur(){
		delete it_;
	}

} /* end namespace */
