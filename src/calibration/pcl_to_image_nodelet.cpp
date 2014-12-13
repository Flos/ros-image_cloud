#include "image_cloud/calibration/pcl_to_image_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Pcl_to_image_nodelet, image_cloud::Pcl_to_image_nodelet, nodelet::Nodelet)

namespace image_cloud {

void
Pcl_to_image_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();
	nh.param<std::string>("name", node_name_, "image2pointcloud_nodelet");
	nh.param<std::string>("sub_pcl", subscribe_topic_pcl_, "");
	nh.param<std::string>("sub_img_info", subscribe_topic_img_info_, "");

	// Optinal
	nh.param<std::string>("pub_img", publish_image_topic_, subscribe_topic_pcl_ + "_mono8");
	nh.param<std::string>("image_frame_id", image_frame_id_, "");
	nh.param<double>("resize_x", resize_faktor_x_, 1);
	nh.param<double>("resize_y", resize_faktor_y_, 1);

	// 2. Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "sub_pcl: \t%s", subscribe_topic_pcl_.c_str());
	ROS_INFO_NAMED(node_name_, "sub_img_info: \t%s", subscribe_topic_img_info_.c_str());
	// Optional
	ROS_INFO_NAMED(node_name_, "pub_img: \t%s:", publish_image_topic_.c_str());
	ROS_INFO_NAMED(node_name_, "resize_width:\t%f", resize_faktor_x_);
	ROS_INFO_NAMED(node_name_, "resize_height:\t%f", resize_faktor_y_);
	ROS_INFO_NAMED(node_name_, "opt, image_frame_id:\t%s", image_frame_id_.c_str());

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
	image_info_sub = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, subscribe_topic_img_info_, queue_size);
	pointcloud_sub = new message_filters::Subscriber<PointCloud>(nh, subscribe_topic_pcl_, queue_size);

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync = new message_filters::Synchronizer<Image_to_cloud_sync>(Image_to_cloud_sync(queue_size), *image_info_sub, *pointcloud_sub);
	sync->registerCallback(boost::bind(&Pcl_to_image_nodelet::callback, this, _1, _2));

	it_ = new image_transport::ImageTransport(nh);
	pub_ = it_->advertise(publish_image_topic_.c_str(), 1);
}


void
Pcl_to_image_nodelet::callback(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	ROS_INFO_NAMED(node_name_,"callback");

	if( pub_.getNumSubscribers() == 0 ){ // dont do anything if no one is interessted
		return;
	}

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		ROS_DEBUG_NAMED(node_name_, "input cloud empty");
		return;
	}

	//Look up transform for cameraladybug_camera4


	if(image_frame_id_.empty()){
		image_frame_id_ = input_msg_image_info->header.frame_id;
	}


	listener_pointcloud_transform.waitForTransform(image_frame_id_.c_str(), //target frame
			input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
			input_msg_image_info->header.stamp, // target time
			ros::Duration(5.0)
			);

	ROS_INFO_NAMED(node_name_,"pointcloud2 w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr,cloud);

	// Transform to odom
	// todo: Transform at pcl time to odom
	if (!pcl_ros::transformPointCloud("/odom", cloud, cloud, listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", "/odom");
	     return;
	}

	// Todo: check if Transform at image time to image_frame_id is right
	cloud.header.stamp = input_msg_image_info->header.stamp.toNSec();

	// Transform to image_frame_id
	if (!pcl_ros::transformPointCloud(image_frame_id_.c_str(), cloud, cloud, listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", image_frame_id_.c_str());
		 return;
	}

	// If we are here we can start with the projection
	sensor_msgs::CameraInfo cam_info(*input_msg_image_info);
	cam_info.P[2] = resize_faktor_x_ * input_msg_image_info->P[2];	// Move the center of the projection the resized image centerx
	cam_info.P[6] = resize_faktor_y_ * input_msg_image_info->P[6]; // Move the center of the projection the resized image centery

	camera_model.fromCameraInfo(cam_info);

	ROS_INFO_NAMED(node_name_,"camera_model:  tx: %f, ty: %f, cx: %f, cy: %f, fx: %f, fy: %f ", camera_model.Tx(), camera_model.Ty(), camera_model.cx(), camera_model.cy(), camera_model.fx(), camera_model.fy() );

	cv::Point2d point_image;

	cv::Mat image;

	image = cv::Mat::zeros(input_msg_image_info->height*resize_faktor_y_, input_msg_image_info->width*resize_faktor_x_, CV_8UC1);
	cv_bridge::CvImage image_pcl(input_msg_image_info->header, sensor_msgs::image_encodings::MONO8, image);

   int i = 0;
   BOOST_FOREACH (const pcl::PointXYZI& pt, cloud.points) {
		// look up 3D position
		// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		// project to 2D Image
	   if( pt.z > 1){ // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x  > 0
				&&  point_image.x < image_pcl.image.cols )
				&& ( point_image.y > 0
				&&  point_image.y < image_pcl.image.rows )
				)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, 1, cv::Scalar(pt.intensity));
			}
			// add colored pixel to cloud_color
	   }
   }

   pub_.publish(image_pcl.toImageMsg());
   ROS_INFO_NAMED(node_name_,"callback end");
}

Pcl_to_image_nodelet::~Pcl_to_image_nodelet(){
	delete image_info_sub;
	delete pointcloud_sub;
	delete it_;
	delete sync;
	}

} /* end namespace */
