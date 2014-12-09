#include "image_cloud/image_enhancement.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Image_enhancement, image_cloud::Image_enhancement, nodelet::Nodelet)

namespace image_cloud {

void
Image_enhancement::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image_enhancement_nodelet");
	nh.param<std::string>("sub", subscribe_topic_, "");
	nh.param<std::string>("pub", publish_topic_, subscribe_topic_ + "_enhanced");
	nh.param<std::string>("filter", filter_, "default");


	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub: \t%s", subscribe_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t\t%s", publish_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "filter: \t%s", filter_.c_str());

	if(subscribe_topic_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}

	it_ = new image_transport::ImageTransport(nh);
	sub_ = it_->subscribe(subscribe_topic_, 1, &Image_enhancement::callback, this);
	pub_ = it_->advertise(publish_topic_, 1);
}


void
Image_enhancement::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
	ROS_INFO_NAMED(node_name_,"callback");



	cv_bridge::CvImagePtr cv_ptr;
	try{
	   cv_ptr = cv_bridge::toCvCopy(input_msg_image, input_msg_image->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
	   ROS_ERROR("cv_bridge exception: %s", e.what());
	   return;
	}

	// READ RGB color image and convert it to Lab
	cv::Mat lab_image;
	cv::cvtColor(cv_ptr->image, lab_image, CV_BGR2Lab);

	// Extract the L channel
	std::vector<cv::Mat> lab_planes(3);
	cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]
	cv::Mat dst;

	// apply the CLAHE algorithm to the L channel
	if(true){
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
		clahe->setClipLimit(4);

		clahe->apply(lab_planes[0], dst);
	}else{
		lab_planes[0].convertTo(dst, -1, 4, 0);
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
				clahe->setClipLimit(4);

				clahe->apply(lab_planes[0], dst);
	}
	// Merge the the color planes back into an Lab image
	dst.copyTo(lab_planes[0]);
	cv::merge(lab_planes, lab_image);

	// convert back to RGB
	cv::cvtColor(lab_image, cv_ptr->image, CV_Lab2BGR);

	pub_.publish(cv_ptr->toImageMsg());

	ROS_INFO_NAMED(node_name_,"callback end");
}

Image_enhancement::~Image_enhancement(){
		delete it_;
	}

} /* end namespace */
