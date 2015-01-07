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
	nh.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_erode");

	it_.reset(new image_transport::ImageTransport(nh));
	sub_ = it_->subscribe(config_.subscribe_topic, 1,
			&Blur::callback, this);
	pub_ = it_->advertise(config_.publish_topic, 1);

	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Blur::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Blur::callback(const sensor_msgs::ImageConstPtr& input_msg_image){
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

	cv::Mat image;

	image = cv::Mat::zeros(cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->image.type());
	cv_bridge::CvImage image_blur(cv_ptr->header, input_msg_image->encoding, image);


	switch(config_.filter){
		case 0:
			cv::bilateralFilter ( cv_ptr->image, image_blur.image, config_.kernel_size, config_.kernel_size*2, config_.kernel_size/2 );
			break;
	    case 1:
			cv::blur( cv_ptr->image, image_blur.image, cv::Size( config_.kernel_size, config_.kernel_size ), cv::Point(-1,-1) );
			break;
	    case 2:
			cv::GaussianBlur( cv_ptr->image, image_blur.image, cv::Size( config_.kernel_size, config_.kernel_size ), 0, 0 );
			break;
	    case 3:
	    	cv::medianBlur ( cv_ptr->image, image_blur.image, config_.kernel_size );
	    	break;
	    default :
	    	ROS_ERROR_NAMED(node_name_, "Filter not implemented, select filter between 0 and 3: instead of %d", config_.filter);
	    	return;
	}

	pub_.publish(image_blur.toImageMsg());

	NODELET_DEBUG("callback end");
}

void
Blur::reconfigure_callback(Config &config, uint32_t level) {
	// Info
	NODELET_INFO("name:\t%s", node_name_.c_str());
	NODELET_INFO("sub:\t%s", config.subscribe_topic.c_str());
	NODELET_INFO("pub:\t%s", config.publish_topic.c_str());
	NODELET_INFO("kernel: \t%i", config.kernel_size);
	NODELET_INFO("filter: \t%i", config.filter);

	if(config.subscribe_topic != config_.subscribe_topic){
	  sub_ = it_->subscribe(config.subscribe_topic, 1,
				&Blur::callback, this);
	  NODELET_INFO("Subscribe topic changed from %s to %s", config_.subscribe_topic.c_str(), config.subscribe_topic.c_str());
	  //
	}

	if(config.publish_topic != config_.publish_topic)
	{
	  pub_ = it_->advertise(config.publish_topic, 1);
	  NODELET_INFO("Publish topic changed from %s to %s", config_.publish_topic.c_str(), config.publish_topic.c_str());
	}
	config_ = config;
}

Blur::~Blur(){
	}

} /* end namespace */
