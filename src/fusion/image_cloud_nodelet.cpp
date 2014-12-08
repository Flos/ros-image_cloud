#include "image_cloud/image_cloud_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Image_cloud_nodelet, image_cloud::Image_cloud_nodelet, nodelet::Nodelet)

namespace image_cloud {

void
Image_cloud_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image2pointcloud_nodelet");
	nh.param<std::string>("sub_pcl", subscribe_topic_pcl_, "");
	nh.param<std::string>("sub_img", subscribe_topic_img_, "");
	nh.param<std::string>("sub_img_info", subscribe_topic_img_info_, "");
	nh.param<std::string>("pub", publish_pcl_topic_, subscribe_topic_pcl_ + "_color");
	nh.param<std::string>("filter", filter_, "default");
	nh.param<std::string>("image_frame_id", image_frame_id_, "");


	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub_pcl: \t%s", subscribe_topic_pcl_.c_str());
	ROS_INFO_NAMED(node_name_, "sub_img: \t%s", subscribe_topic_img_.c_str());
	ROS_INFO_NAMED(node_name_, "sub_img_info: \t%s", subscribe_topic_img_info_.c_str());
	ROS_INFO_NAMED(node_name_, "pub:\t\t%s", publish_pcl_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "filter: \t%s", filter_.c_str());

	if(!image_frame_id_.empty()){
		ROS_INFO_NAMED(node_name_, "override image frame id: \t%s", image_frame_id_.c_str());
	}

	if(subscribe_topic_img_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
		return;
	}


	if(subscribe_topic_img_info_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no camera_info subscribe topic defined");
		return;
	}

	if(subscribe_topic_pcl_.empty()) {
		ROS_ERROR_NAMED(node_name_, "no pcl subscribe topic defined");
		return;
	}



	//Filter messages
	int queue_size = 30;
	image_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, subscribe_topic_img_, queue_size);
	image_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, subscribe_topic_img_info_, queue_size);
	pointcloud_sub = new message_filters::Subscriber<PointCloud>(nh, subscribe_topic_pcl_, queue_size);

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync = new message_filters::Synchronizer<Image_to_cloud_sync>(Image_to_cloud_sync(queue_size), *image_sub, *image_info_sub, *pointcloud_sub);
	sync->registerCallback(boost::bind(&Image_cloud_nodelet::callback, this, _1, _2, _3));
}


void
Image_cloud_nodelet::callback(const sensor_msgs::ImageConstPtr& input_msg_image, const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	ROS_INFO_NAMED(node_name_,"callback");

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		ROS_DEBUG_NAMED(node_name_, "input cloud empty");
		return;
	}
	if(input_msg_image->height == 0 || input_msg_image->width == 0){
		ROS_DEBUG_NAMED(node_name_, "input image empty");
		return;
	}

	//Look up transform for cameraladybug_camera4

	if(image_frame_id_.empty()){
		image_frame_id_ = input_msg_image->header.frame_id;
	}


	listener_pointcloud_transform.waitForTransform(image_frame_id_.c_str(), //target frame
			input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
			input_msg_image->header.stamp, // target time
			ros::Duration(5.0)
			);

	//	TF pcl to odom frame, time stamp pcl
	//  TF pcl to camera frame, time stamp camera

	try{
		listener_pointcloud_transform.lookupTransform(image_frame_id_.c_str(), //target frame
				input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
				input_msg_image->header.stamp, // time
				tf_pointcloud_to_camera_position  // result tf
				);

	}catch(tf::ExtrapolationException &e){
		return;
	};

	// If we are here we can start
	//camera_model.fromCameraInfo(input_msg_image_info);

	//camera_model.project3dToPixel()

	ROS_INFO_NAMED(node_name_,"pointcloud w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );
	//ROS_INFO_NAMED(node_name_,"pointcloud w: %i \th: %i \t\ttime: %lu",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp );
	ROS_INFO_NAMED(node_name_,"image      w: %i \th: %i \ttime: %i.%i",input_msg_image->width, input_msg_image->height, input_msg_image->header.stamp.sec, input_msg_image->header.stamp.nsec );

//	pcl::PCLPointCloud2 cloud2;
//	pcl_conversions::toPCL(*input_msg_cloud_ptr, cloud2);
//
//	pcl::PointCloud<pcl::PointXYZ> cloud;
//	pcl::fromPCLPointCloud2(cloud2, cloud);
//
//
//	if (!pcl_ros::transformPointCloud("/odom", cloud, cloud, listener_pointcloud_transform)) {
//			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", "/odom");
//	     return;
//	}


	ROS_INFO_NAMED(node_name_,"callback end");
}

Image_cloud_nodelet::~Image_cloud_nodelet(){
		delete image_sub;
		delete pointcloud_sub;
		delete sync;
	}

} /* end namespace */
