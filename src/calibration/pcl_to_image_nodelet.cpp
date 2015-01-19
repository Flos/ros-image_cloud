#include "calibration/pcl_to_image_nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <common/time.hpp>
#include <common/pcl_features.hpp>
#include <common/project2d.hpp>
#include <common/depth_filter.hpp>
#include <common/filter_depth_intensity.hpp>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Pcl_to_image_nodelet, image_cloud::Pcl_to_image_nodelet, nodelet::Nodelet)

namespace image_cloud {


void
Pcl_to_image_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh = getPrivateNodeHandle();

	init_params();

	init_sub();
	it_.reset( new image_transport::ImageTransport(nh));
	init_pub();

	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Pcl_to_image_nodelet::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
	listener_pointcloud_transform.reset(new tf::TransformListener(nh, ros::Duration(config_.tf_buffer_length), true));
}

void
Pcl_to_image_nodelet::init_params(){
	nh.param<std::string>("node_name", node_name_, "pcl_to_image");

	nh.param<std::string>("subscribe_topic_pcl", config_.subscribe_topic_pcl, "");
	nh.param<std::string>("subscribe_topic_img_info",  config_.subscribe_topic_img_info, "");
	nh.param<std::string>("publish_topic", config_.publish_topic, "");
	nh.param<std::string>("publish_topic_pcl", config_.publish_topic_pcl, "");
	nh.param<std::string>("image_tf_frame_id", config_.image_tf_frame_id, "");

	nh.param<bool>("use_reference", config_.use_reference, false);
	nh.param<std::string>("reference_frame", config_.reference_frame, "");
	nh.param<int>("point_size", config_.point_size, 1);
	nh.param<int>("tf_buffer_length", config_.tf_buffer_length, 30);
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
	NODELET_INFO( "name:\t\t%s", node_name_.c_str());

	NODELET_INFO( "subscribe_topic_pcl:\t%s", config_.subscribe_topic_pcl.c_str());
	NODELET_INFO( "subscribe_topic_img_info:\t%s", config_.subscribe_topic_img_info.c_str());
	NODELET_INFO( "publish_topic: \t%s", config_.publish_topic.c_str());
	NODELET_INFO( "publish_topic_pcl: \t%s", config_.publish_topic_pcl.c_str());
	NODELET_INFO( "image_tf_frame_id:\t%s", config_.image_tf_frame_id.c_str());

	//NODELET_INFO( "use_reference: \t %s", config_.use_reference, config_.use_reference ? "true" : "false" );
	NODELET_INFO( "reference_frame:\t%s:", config_.reference_frame.c_str());
	NODELET_INFO( "point_size:\t%i", config_.point_size);
	NODELET_INFO( "tf_buffer_length:\t%i", config_.tf_buffer_length);
	NODELET_INFO( "resize_faktor_x:\t%f", config_.resize_faktor_x);
	NODELET_INFO( "resize_faktor_y:\t%f", config_.resize_faktor_y);

	NODELET_INFO( "queue_size:\t%i", config_.queue_size);
	NODELET_INFO( "normal_search_radius:\t%f", config_.normal_search_radius);
	NODELET_INFO( "feature:\t%i", config_.feature);
}

void Pcl_to_image_nodelet::init_sub() {
	NODELET_INFO( "init_sub");
//	image_info_sub.reset();
//	pointcloud_sub.reset();
//	sync.reset();
	image_info_sub.reset(
			new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh,
					config_.subscribe_topic_img_info, config_.queue_size));
	pointcloud_sub.reset(
			new message_filters::Subscriber<PointCloud>(nh,
					config_.subscribe_topic_pcl, config_.queue_size));
	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync.reset(
			new message_filters::Synchronizer<Image_to_cloud_sync>(
					Image_to_cloud_sync(config_.queue_size), *image_info_sub,
					*pointcloud_sub));
	sync->registerCallback(
			boost::bind(&Pcl_to_image_nodelet::callback, this, _1, _2));
}

void
Pcl_to_image_nodelet::init_pub() {
	NODELET_INFO("init_pub");
	//Filter messages
	pub_ = it_->advertise(config_.publish_topic.c_str(), 1);
	pub_depth_ = it_->advertise(config_.publish_topic + "_depth", 1);
	pub_cloud_ = nh.advertise<PointCloud>(config_.publish_topic_pcl +"_transformed",1);
	pub_cloud_filtred_ = nh.advertise<PointCloud>(config_.publish_topic_pcl.c_str(),1);
	//pub_range_ = it_->advertise(config_.subscribe_topic_pcl + "_depth", 1);
}

void
Pcl_to_image_nodelet::reconfigure_callback(Config &config, uint32_t level){
	NODELET_INFO("reconfiguration_callback");
	config_lock_.lock();
	bool reset_sub = false;
	bool reset_pub = false;

	if(config.subscribe_topic_pcl != config_.subscribe_topic_pcl
		|| config.subscribe_topic_img_info != config_.subscribe_topic_img_info
		)
	 {
		reset_sub = true;
	 }

	if(config.publish_topic != config_.publish_topic
		|| config.publish_topic_pcl != config_.publish_topic_pcl)
	{
		reset_pub = true;
	}

	 config_ = config;
	 params();

	 if(reset_sub){
		 // TODO: Fix boost::lock_error on init_sub() after reconfiguration
		 // Error Message: Reconfigure callback failed with exception boost::lock_error
		 NODELET_WARN("Changing subscriber not implemented, change subscriber in launch file and restart node");
		 //init_sub();
	 }

	 if(reset_pub){
		 init_pub();
	 }
	 config_lock_.unlock();
}



void
Pcl_to_image_nodelet::callback(const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	NODELET_DEBUG("Pcl_to_image_nodelet: callback");

	time("pcl_to_image: ", input_msg_image_info->header, input_msg_cloud_ptr->header, true);

	if( pub_.getNumSubscribers() == 0
			&& pub_depth_.getNumSubscribers() == 0){ // don't do anything if no one is interested
		return;
	}

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		NODELET_WARN("input cloud empty");
		return;
	}

	if(!config_lock_.try_lock()){
		NODELET_ERROR("callback locked");
		return;
	}

	//Look up transform for cameraladybug_camera4
	if(config_.image_tf_frame_id.empty()){
		config_.image_tf_frame_id = input_msg_image_info->header.frame_id;
	}

	std::string tf_error_msg;
	if(!listener_pointcloud_transform->waitForTransform(config_.image_tf_frame_id.c_str(), //target frame
														input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
														input_msg_image_info->header.stamp, // target time
														//ros::Time(0),
														ros::Duration(5.0),
														ros::Duration(0.1),
														&tf_error_msg
														)
	)
	{
		NODELET_ERROR("%s", tf_error_msg.c_str() );
		config_lock_.unlock();
		return;
	}

	NODELET_DEBUG("pointcloud2 w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr, cloud);

	if(config_.use_reference && !config_.reference_frame.empty())
	{
		// Transform to odom
		// todo: Transform at pcl time to odom
		if (!pcl_ros::transformPointCloud( config_.reference_frame, cloud, cloud, *listener_pointcloud_transform)) {
				NODELET_ERROR("Cannot transform point cloud to the fixed frame %s", config_.reference_frame.c_str());
				config_lock_.unlock();
				return;
		}
	}

	// Todo: check if Transform at image time to image_frame_id is right
	cloud.header.stamp = input_msg_image_info->header.stamp.toNSec();


	// Transform to image_frame_id
	if (!pcl_ros::transformPointCloud(config_.image_tf_frame_id.c_str(), cloud, cloud, *listener_pointcloud_transform)) {
			NODELET_ERROR("Cannot transform point cloud to the fixed frame %s", config_.image_tf_frame_id.c_str());
			config_lock_.unlock();
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


	pcl::PointCloud<pcl::PointXY> cloud2d;
	pcl::PointCloud<pcl::PointXYZI> cloud3d_hit_image;
	pcl::PointCloud<pcl::PointXYZI> cloud3d_filtred;
	cloud3d_filtred.header = cloud.header;
	cloud3d_hit_image.header = cloud.header;
	try{
		switch(config_.feature){
			default:
			case 0: // Intensity
				extract_intensity(camera_model, cloud.makeShared(), image_pcl, image_depth, config_.point_size);
				break;
			case 1: // Normals
				{
					//	extract_normals(camera_model, cloud.makeShared(), image_pcl, image_depth, config_.point_size, config_.normal_search_radius);

					std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(image.rows, std::vector<boost::shared_ptr<pcl::PointXYZI> > (image.cols));

					project2d::project_2d(camera_model, cloud, map, image.rows, image.cols);
					filter::filter_depth_intensity(map, cloud3d_filtred,
							0.3, // depth
							50, // intensity
							2,	// neighbors
							false); // search direction_x?
					project2d::project_2d(map, image_pcl.image, project2d::INTENSITY, config_.point_size);
				}
				break;
			case 2: // Intesity + Normals
				//extract_intensity_and_normals(camera_model, cloud.makeShared(), image_pcl, image_depth, config_.point_size, config_.normal_search_radius);

				project2d::project_2d(camera_model, cloud, cloud2d, cloud3d_hit_image, image_pcl.image.rows, image_pcl.image.cols );
				filter_depth_edges(cloud2d.makeShared(), cloud3d_hit_image.makeShared(), cloud3d_filtred, image_pcl );
				break;
			case 3: // depth discontinuity
				extract_depth_discontinuity(camera_model, cloud.makeShared(), image_pcl, config_.point_size);
				break;
		}
	}catch(std::exception &e){
		NODELET_ERROR("Exception: %s", e.what());
		config_lock_.unlock();
		return;
	}

   pub_.publish(image_pcl.toImageMsg());
   pub_depth_.publish(image_depth.toImageMsg());


   if(config_.use_reference && !config_.reference_frame.empty())
   	{
   		// Transform to odom
   		if (!pcl_ros::transformPointCloud( config_.reference_frame, cloud3d_filtred, cloud3d_filtred, *listener_pointcloud_transform)) {
   				NODELET_ERROR("Cannot transform point cloud to the fixed frame %s", config_.reference_frame.c_str());
   				config_lock_.unlock();
   				return;
   		}
   	}

   // dosent work! stamp is empty (0) after publish
   //cloud3d_filtered.header.stamp = input_msg_image_info->header.stamp.toNSec();

   PointCloud msg_cloud3d_filtred;

   pcl::toROSMsg(cloud3d_filtred, msg_cloud3d_filtred);
   msg_cloud3d_filtred.header.stamp = input_msg_image_info->header.stamp;
   msg_cloud3d_filtred.height = 1;
   msg_cloud3d_filtred.width = cloud3d_filtred.points.size();

   pub_cloud_filtred_.publish(msg_cloud3d_filtred);


   PointCloud msg_cloud3d_transformed;

   pcl::toROSMsg(cloud3d_hit_image, msg_cloud3d_transformed);
   msg_cloud3d_transformed.header.stamp = input_msg_image_info->header.stamp;
   msg_cloud3d_transformed.height = 1;
   msg_cloud3d_transformed.width = cloud3d_hit_image.points.size();

   pub_cloud_.publish(msg_cloud3d_transformed);

   NODELET_DEBUG("callback end");
   config_lock_.unlock();
}

Pcl_to_image_nodelet::~Pcl_to_image_nodelet(){
	}

} /* end namespace */
