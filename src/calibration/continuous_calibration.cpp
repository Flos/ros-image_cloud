/*
 * auto_calibration.cpp
 *
 *  Created on: 29.01.2015
 *      Author: fnolden
 */

#include <image_cloud/calibration/continuous_calibration.h>

namespace image_cloud {

Continuous_calibration::Continuous_calibration() {
}

Continuous_calibration::Continuous_calibration(ros::NodeHandle &nh, ros::NodeHandle &nh_private) {
	init(nh, nh_private);
}

Continuous_calibration::~Continuous_calibration() {
	// TODO Auto-generated destructor stub
}

void
Continuous_calibration::callback_info(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info){
	data.camera_model.fromCameraInfo(input_msg_image_info);
	data.is_camera_model_set = true;
	sub_info.shutdown();
}

void
Continuous_calibration::callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
	// pipeline..
	// 1. Image processing
	cv_bridge::CvImagePtr cv_ptr;
	try{
	   cv_ptr = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
	   ROS_ERROR("cv_bridge exception: %s", e.what());
	   return;
	}

	// Create edged, inverse transformed image
	cv::Mat inverse_transformed;
	create_inverse_transformed(cv_ptr->image, inverse_transformed);

	// Pointcloud
	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*cloud_msg, cloud);

	// Transform pointcloud
	std::string error_string;
	std::string image_frame_id;

	if(config.frame_id_image.empty()){
		image_frame_id = image_msg->header.frame_id;
	}


	if(!transform_pointcloud<pcl::PointXYZI>(	*tf_listener,
												cloud,
												image_frame_id,
												image_msg->header.stamp,
												cloud_msg->header.frame_id,
												cloud_msg->header.stamp,
												error_string
												)){
		ROS_WARN("Auto_calibration: %s", error_string.c_str());
		return;
	}

	// Push back data;

	data.window.push_back(inverse_transformed, cloud);

	// is window  size reached ?
	if(data.window.size() == config.windows_size){
		// Do calibration here

		search::get_best_tf< pcl::PointXYZI, uchar>(
								data.calibration,  // tf in
								data.calibration,  // best tf out
								data.camera_model, // camera model
								data.window.pointclouds, // pointclouds
								data.window.images, // images
								config.grid_search_radius, // radious of the search
								config.grid_search_steps); // steps

		// Transform to current best calibration
		transform_pointcloud<pcl::PointXYZI>(cloud, data.calibration);

		// Publish calibration TF
		std::string frame_id_calibration;
		if(config.frame_id_calibration.empty()){
			frame_id_calibration = frame_id_calibration +"_calibrated";
		}
		else{
			frame_id_calibration = config.frame_id_calibration;
		}

		pub_tf.publish(data.calibration, image_frame_id, frame_id_calibration, image_msg->header.stamp);

		if(!config.publish_pointcloud_topic.empty()){
			PointCloud2_msg msg_cloud;

			pcl::toROSMsg(cloud, msg_cloud);

			msg_cloud.header.stamp = image_msg->header.stamp;
			msg_cloud.header.frame_id = frame_id_calibration;
			msg_cloud.height = 1;
			msg_cloud.width = cloud.points.size();

			pub_cloud.publish(cloud);
		}

		if(!config.publish_pointcloud_color_topic.empty()){
			PointCloudColor cloud_color;
			PointCloud2_msg msg_cloud_color;

			pointcloud_rgb<pcl::PointXYZI, cv::Vec3b>(data.camera_model, cloud, cv_ptr->image, cloud_color, config.fusion_min_color);

		    pcl::toROSMsg(cloud_color, msg_cloud_color);
		    msg_cloud_color.header.stamp = image_msg->header.stamp;
		    msg_cloud_color.header.frame_id = frame_id_calibration;
		    msg_cloud_color.height = 1;
		    msg_cloud_color.width = cloud_color.points.size();

			pub_cloud.publish(msg_cloud_color);
		}
	}
}

void
Continuous_calibration::reconfigure_callback(Config &config, uint32_t level){

	bool init_subscriber = false;
	bool init_publisher = false;

	if( this->config.subscribe_pcl_topic != config.subscribe_pcl_topic
		|| this->config.subscribe_image_topic != config.subscribe_image_topic
		|| this->config.subscribe_image_info_topic != config.subscribe_image_info_topic)
	{
		init_subscriber = true;
	}

	if( this->config.publish_pointcloud_topic != config.publish_pointcloud_topic
		|| this->config.publish_pointcloud_color_topic != config.publish_pointcloud_color_topic)
	{
		init_publisher = true;
	}

	this->config = config;

	init_params();
	print_params();

	if(init_publisher){
		init_pub();
	}

	if(init_subscriber){
		init_sub();
	}

}

void
Continuous_calibration::init(ros::NodeHandle &nh, ros::NodeHandle &nh_private){
	this->nh = nh;
	this->nh_private = nh_private;

	// Set up dynamic reconfigure
	reconfigure_server.reset(new ReconfigureServer(nh_private));
	ReconfigureServer::CallbackType f = boost::bind(&Continuous_calibration::reconfigure_callback, this, _1, _2);
	reconfigure_server->setCallback(f);

	tf_listener.reset(new tf::TransformListener(nh, ros::Duration(config.tf_buffer_length), true));
}

void
Continuous_calibration::init_params(){
	// Nothing to do here
	data.window.window_size = config.windows_size;
}

void
Continuous_calibration::init_sub(){
	sub_image.reset(
			new message_filters::Subscriber<sensor_msgs::Image>(nh,
					config.subscribe_image_topic, config.queue_size));
	sub_pointcloud.reset(
			new message_filters::Subscriber<PointCloud2_msg>(nh,
					config.subscribe_pcl_topic, config.queue_size));
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync_filter.reset(
			new message_filters::Synchronizer<Sync_filter>(
					Sync_filter(config.queue_size),
					*sub_image,
					*sub_pointcloud));
	sync_filter->registerCallback(
			boost::bind(&Continuous_calibration::callback, this, _1, _2));

	sub_info = nh.subscribe<sensor_msgs::CameraInfo>(config.subscribe_image_info_topic, 1, boost::bind(&Continuous_calibration::callback_info, this, _1) );
}

void
Continuous_calibration::init_pub(){
	if(!config.publish_pointcloud_topic.empty()){
		pub_cloud = nh.advertise<PointCloud2_msg>(config.publish_pointcloud_topic, 1);
	}
	if(!config.publish_pointcloud_color_topic.empty()){
		pub_cloud_color = nh.advertise<PointCloudColor>(config.publish_pointcloud_color_topic, 1);
	}
}

void
Continuous_calibration::print_params(){
	ROS_INFO("Auto_calibration: subscribe_pcl_topic:\t\t %s", config.subscribe_pcl_topic.c_str());
	ROS_INFO("Auto_calibration: subscribe_image_topic:\t\t %s", config.subscribe_image_topic.c_str());
	ROS_INFO("Auto_calibration: subscribe_image_info_topic:\t %s", config.subscribe_image_info_topic.c_str());

	ROS_INFO("Auto_calibration: \t %s", config.publish_pointcloud_topic.c_str());
	ROS_INFO("Auto_calibration: \t %s", config.publish_pointcloud_color_topic.c_str());

	ROS_INFO("Auto_calibration: \t %s", config.frame_id_image.c_str());
	ROS_INFO("Auto_calibration: \t %s", config.frame_id_calibration.c_str());

	ROS_INFO("Auto_calibration: windows_size:\t %d", config.windows_size);
	ROS_INFO("Auto_calibration: queue_size:  \t %d", config.queue_size);
	ROS_INFO("Auto_calibration: tf_buffer_length:\t %d", config.tf_buffer_length);

	ROS_INFO("Auto_calibration: grid_search_radius:\t %f", config.grid_search_radius);
	ROS_INFO("Auto_calibration: grid_search_steps: \t %d", config.grid_search_steps);

	ROS_INFO("Auto_calibration: fusion_min_color: \t %d", config.fusion_min_color);
}

} /* namespace image_cloud */
