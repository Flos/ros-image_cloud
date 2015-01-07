#include "calibration/pcl_to_image_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Pcl_to_image_nodelet, image_cloud::Pcl_to_image_nodelet, nodelet::Nodelet)

namespace image_cloud {


void
Pcl_to_image_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();

	init_params();

	it_.reset( new image_transport::ImageTransport(nh));
	init_pubsub();


	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Pcl_to_image_nodelet::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Pcl_to_image_nodelet::init_params(){
	nh.param<std::string>("node_name", node_name_, "pcl_to_image");

	nh.param<std::string>("subscribe_topic_pcl", config_.subscribe_topic_pcl, "");
	nh.param<std::string>("subscribe_topic_img_info",  config_.subscribe_topic_img_info, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, "");
	nh.param<std::string>("image_tf_frame_id", config_.image_tf_frame_id, "");

	nh.param<bool>("use_reference", config_.use_reference, false);
	nh.param<std::string>("reference_frame", config_.reference_frame, "");
	nh.param<int>("point_size", config_.point_size, 1);
	nh.param<double>("resize_faktor_x", config_.resize_faktor_x, 1);
	nh.param<double>("resize_faktor_y", config_.resize_faktor_y, 1);

	nh.param<int>("queue_size", config_.queue_size, 30);
	nh.param<double>("normal_search_radius", config_.normal_search_radius, 0.2);
	nh.param<int>("feature", config_.feature, 0);

	params();
}

void
Pcl_to_image_nodelet::params(){
	// Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());

	ROS_INFO_NAMED(node_name_, "subscribe_topic_pcl:\t%s", config_.subscribe_topic_pcl.c_str());
	ROS_INFO_NAMED(node_name_, "subscribe_topic_img_info:\t%s", config_.subscribe_topic_img_info.c_str());
	ROS_INFO_NAMED(node_name_, "publish_topic: \t%s", config_.publish_topic.c_str());
	ROS_INFO_NAMED(node_name_, "image_tf_frame_id:\t%s", config_.image_tf_frame_id.c_str());

	ROS_INFO_NAMED(node_name_, "use_reference: \t %s", config_.use_reference, config_.use_reference ? "true" : "false" );
	ROS_INFO_NAMED(node_name_, "reference_frame:\t%s:", config_.reference_frame.c_str());
	ROS_INFO_NAMED(node_name_, "point_size:\t%i", config_.point_size);
	ROS_INFO_NAMED(node_name_, "resize_faktor_x:\t%f", config_.resize_faktor_x);
	ROS_INFO_NAMED(node_name_, "resize_faktor_y:\t%f", config_.resize_faktor_y);

	ROS_INFO_NAMED(node_name_, "queue_size:\t%i", config_.queue_size);
	ROS_INFO_NAMED(node_name_, "normal_search_radius:\t%f", config_.normal_search_radius);
	ROS_INFO_NAMED(node_name_, "feature:\t%i", config_.feature);
}

void
Pcl_to_image_nodelet::init_pubsub() {
	ROS_INFO_NAMED(node_name_, "init_pubsub");
	//Filter messages
	image_info_sub.reset( new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, config_.subscribe_topic_img_info, config_.queue_size));
	pointcloud_sub.reset( new message_filters::Subscriber<PointCloud>(nh, config_.subscribe_topic_pcl, config_.queue_size));

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync.reset( new message_filters::Synchronizer<Image_to_cloud_sync>(Image_to_cloud_sync(config_.queue_size), *image_info_sub, *pointcloud_sub));
	sync->registerCallback(boost::bind(&Pcl_to_image_nodelet::callback, this, _1, _2));

	pub_ = it_->advertise(config_.publish_topic.c_str(), 1);
	pub_depth_ = it_->advertise(config_.publish_topic + "_depth", 1);
	//pub_range_ = it_->advertise(config_.subscribe_topic_pcl + "_depth", 1);
}

void
Pcl_to_image_nodelet::reconfigure_callback(Config &config, uint32_t level){
	ROS_INFO_NAMED(node_name_, "reconfiguration_callback");
	config_lock_.lock();
	bool reset = false;

	if(config.subscribe_topic_pcl != config_.subscribe_topic_pcl
		|| config.publish_topic != config_.publish_topic
		|| config.subscribe_topic_img_info != config_.subscribe_topic_img_info
		)
	 {
		reset = true;
	 }

	 config_ = config;
	 params();

	 if(reset){
		 init_pubsub();
	 }
	 config_lock_.unlock();
}

void Pcl_to_image_nodelet::extract_intensity(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv_bridge::CvImage& image_pcl, cv_bridge::CvImage& image_depth) {
	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZI& pt, (*cloud).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, config_.point_size, cv::Scalar(pt.intensity), -1);
				cv::circle(image_depth.image, point_image, config_.point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

void Pcl_to_image_nodelet::extract_normals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv_bridge::CvImage& image_pcl, cv_bridge::CvImage& image_depth) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals = calculate_normals(cloud);
	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZINormal& pt, (*cloud_normals).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, config_.point_size, cv::Scalar(pt.data_n[0]/3 + pt.data_n[1]/3 + pt.data_n[2]/3 ), -1);
				cv::circle(image_depth.image, point_image, config_.point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Pcl_to_image_nodelet::calculate_normals(
		const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n(
			new pcl::search::KdTree<pcl::PointXYZI>());
	// Estimate the normals of the cloud_xyz
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals(
			new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree_n);
	ne.setRadiusSearch(config_.normal_search_radius);
	ne.compute(*cloud_normals);
	return cloud_normals;
}

void Pcl_to_image_nodelet::extract_intensity_and_normals(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, cv_bridge::CvImage& image_pcl, cv_bridge::CvImage& image_depth) {
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_normals = calculate_normals(cloud);

	cv::Point2d point_image;
	BOOST_FOREACH (const pcl::PointXYZINormal& pt, (*cloud_normals).points){
	// look up 3D position
	// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
	// project to 2D Image
		if( pt.z > 1) { // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
							&& point_image.x < image_pcl.image.cols )
					&& ( point_image.y > 0
							&& point_image.y < image_pcl.image.rows )
			)
			{
				// Get image Color
				cv::circle(image_pcl.image, point_image, config_.point_size, cv::Scalar(pt.intensity/2 + (pt.data_n[0]/3 + pt.data_n[1]/3 + pt.data_n[2]/3)/2), -1);
				cv::circle(image_depth.image, point_image, config_.point_size, cv::Scalar(pt.z), -1);
			}
			// add colored pixel to cloud_color
		}
	}
}

void
Pcl_to_image_nodelet::callback(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	NODELET_DEBUG("callback");
	if( pub_.getNumSubscribers() == 0 ){ // dont do anything if no one is interessted
		return;
	}

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		ROS_DEBUG_NAMED(node_name_, "input cloud empty");
		return;
	}

	config_lock_.lock();

	//Look up transform for cameraladybug_camera4
	if(config_.image_tf_frame_id.empty()){
		config_.image_tf_frame_id = input_msg_image_info->header.frame_id;
	}


	if(!listener_pointcloud_transform.waitForTransform(config_.image_tf_frame_id.c_str(), //target frame
														input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
														input_msg_image_info->header.stamp, // target time
														ros::Duration(5.0)
														)
	)
	{
		NODELET_WARN("no valid transform from %s to %s", input_msg_cloud_ptr->header.frame_id.c_str(), config_.image_tf_frame_id.c_str() );
		return;
	}

	NODELET_DEBUG("pointcloud2 w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr, cloud);

	if(config_.use_reference && !config_.reference_frame.empty())
	{
		// Transform to odom
		// todo: Transform at pcl time to odom
		if (!pcl_ros::transformPointCloud( config_.reference_frame, cloud, cloud, listener_pointcloud_transform)) {
				NODELET_WARN("Cannot transform point cloud to the fixed frame %s", config_.reference_frame.c_str());
			 return;
		}
	}

	// Todo: check if Transform at image time to image_frame_id is right
	cloud.header.stamp = input_msg_image_info->header.stamp.toNSec();

	// Transform to image_frame_id
	if (!pcl_ros::transformPointCloud(config_.image_tf_frame_id.c_str(), cloud, cloud, listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s", config_.image_tf_frame_id.c_str());
		 return;
	}

	// If we are here we can start with the projection
	sensor_msgs::CameraInfo cam_info(*input_msg_image_info);
	cam_info.P[2] = config_.resize_faktor_x * input_msg_image_info->P[2];	// Move the center of the projection the resized image centerx
	cam_info.P[6] = config_.resize_faktor_y * input_msg_image_info->P[6]; // Move the center of the projection the resized image centery

	camera_model.fromCameraInfo(cam_info);

	NODELET_DEBUG("camera_model:  tx: %f, ty: %f, cx: %f, cy: %f, fx: %f, fy: %f ", camera_model.Tx(), camera_model.Ty(), camera_model.cx(), camera_model.cy(), camera_model.fx(), camera_model.fy() );


	cv::Mat image;
	image = cv::Mat::zeros(input_msg_image_info->height*config_.resize_faktor_y, input_msg_image_info->width*config_.resize_faktor_x, CV_8UC1);
	cv_bridge::CvImage image_pcl(input_msg_image_info->header, sensor_msgs::image_encodings::MONO8, image);

	cv::Mat mat_image_depth = cv::Mat::zeros(input_msg_image_info->height*config_.resize_faktor_y, input_msg_image_info->width*config_.resize_faktor_x, CV_8UC1);
	cv_bridge::CvImage image_depth(input_msg_image_info->header, sensor_msgs::image_encodings::MONO8, mat_image_depth);

	try{
		switch(config_.feature){
			default:
			case 0: // Intensity
				extract_intensity(cloud.makeShared(), image_pcl, image_depth);
				break;
			case 1: // Normals
				extract_normals(cloud.makeShared(), image_pcl, image_depth);
				break;
			case 2: // Intesity + Normals
				extract_intensity_and_normals(cloud.makeShared(), image_pcl, image_depth);
				break;
		}
	}catch(std::exception &e){
		NODELET_ERROR("Exception: %s", e.what());
		return;
	}

	config_lock_.unlock();
   pub_.publish(image_pcl.toImageMsg());
   pub_depth_.publish(image_depth.toImageMsg());

   // range image
   // Convert the cloud to range image.



//   tf::Transform pose;
//   Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud.sensor_origin_[0],
//   								 cloud.sensor_origin_[1],
//   								 cloud.sensor_origin_[2])) *
//   								 Eigen::Affine3f(cloud.sensor_orientation_);
//   	float noiseLevel = 0.0f, minimumRange = 0.0f;
//   	pcl::RangeImagePlanar rangeImage;
//   	rangeImage.createFromPointCloudWithFixedSize(cloud, image.rows, image.cols,
//   			camera_model.cx(), camera_model.cy(),
//   			camera_model.fx(), camera_model.fy(),
//   			sensorPose, pcl::RangeImage::CAMERA_FRAME,
//   			noiseLevel, minimumRange);

//   	sensor_msgs::Image range_image;
//   //	pcl::toROSMsg(*input_msg_cloud_ptr, range_image);
//
//
//   	// Border extractor object.
////   	pcl::RangeImageBorderExtractor borderExtractor(&rangeImage);
////
////   	borderExtractor.compute(*borders);
// //  	rangeImage.getImagePoint()
//
//   //pub_range_.publish(range_image);

   ROS_DEBUG_NAMED(node_name_,"callback end");
}

Pcl_to_image_nodelet::~Pcl_to_image_nodelet(){
	}

} /* end namespace */
