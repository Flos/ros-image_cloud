#include "fusion/fusion.h"
#include <pluginlib/class_list_macros.h>
#include <common/time.hpp>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Fusion, image_cloud::Fusion, nodelet::Nodelet)

namespace image_cloud {

void Fusion::get_param() {
	nh.param<std::string>("name", 						node_name_,						 	"image2pointcloud_nodelet");
	nh.param<std::string>("subscribe_topic_pcl", 		config_.subscribe_topic_pcl, 		"");
	nh.param<std::string>("subscribe_topic_image", 		config_.subscribe_topic_image, 		"");
	nh.param<std::string>("subscribe_topic_image_info", config_.subscribe_topic_image_info, "");
	nh.param<std::string>("publish_topic_image", 		config_.publish_topic_image, 		config_.subscribe_topic_pcl + "_color");
	nh.param<std::string>("publish_topic_pcl", 			config_.publish_topic_pcl, 			config_.subscribe_topic_image + "_color");
	nh.param<std::string>("image_tf_frame_id", 			config_.image_tf_frame_id, 			"");
	nh.param<std::string>("reference_frame", 			config_.reference_frame, 			"");

	nh.param<int>("min_color", 							config_.min_color, 					 8);
	nh.param<int>("tf_buffer_length", 					config_.tf_buffer_length,		 	30);
	nh.param<int>("queue_size",							config_.queue_size, 				30);
}

void Fusion::print() {
	// Info
	NODELET_INFO("name:\t\t%s", 								node_name_.c_str());
	NODELET_INFO("subscribe_topic_pcl: \t%s", 					config_.subscribe_topic_pcl.c_str());
	NODELET_INFO("subscribe_topic_image: \t%s", 				config_.subscribe_topic_image.c_str());
	NODELET_INFO("subscribe_topic_image_info: \t%s", 			config_.subscribe_topic_image_info.c_str());
	NODELET_INFO("publish_topic_image:\t\t%s", 					config_.publish_topic_image.c_str());
	NODELET_INFO("publish_topic_pcl:\t\t%s", 					config_.publish_topic_pcl.c_str());
	NODELET_INFO("image_tf_frame_id:\t\t%s",					config_.image_tf_frame_id.c_str());
	NODELET_INFO("reference_frame:\t\t%s",						config_.reference_frame.c_str());

	NODELET_INFO("min_color: \t\t\t%d", 						config_.min_color);
	NODELET_INFO("tf_buffer_length: \t\t%d", 					config_.tf_buffer_length);
	NODELET_INFO("queue_size: \t\t\t%d", 						config_.queue_size);


	if (!config_.image_tf_frame_id.empty()) {
		NODELET_INFO("override image frame id: \t%s", config_.image_tf_frame_id.c_str());
	}
	if (!config_.reference_frame.empty()) {
		NODELET_INFO("publish cloud transformed to reference frame: \t%s",
				config_.reference_frame.c_str());
	}
}

void Fusion::init_sub() {
	if (config_.subscribe_topic_pcl.empty()) {
		ROS_ERROR_NAMED(node_name_, "no img subscribe topic defined");
	}
	if (config_.subscribe_topic_image.empty()) {
		ROS_ERROR_NAMED(node_name_, "no camera_info subscribe topic defined");
	}
	if (config_.subscribe_topic_image_info.empty()) {
		ROS_ERROR_NAMED(node_name_, "no pcl subscribe topic defined");
	}
	//Filter messages
	int queue_size = 30;
	image_sub.reset( new message_filters::Subscriber<sensor_msgs::Image>(nh,
			config_.subscribe_topic_image, queue_size));

	image_info_sub.reset( new message_filters::Subscriber<sensor_msgs::CameraInfo>(	nh,
			config_.subscribe_topic_image_info, queue_size));

	pointcloud_sub.reset( new message_filters::Subscriber<PointCloud>(nh,
			config_.subscribe_topic_pcl, queue_size));

	// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
	sync.reset( new message_filters::Synchronizer<Image_to_cloud_sync>(
			Image_to_cloud_sync(queue_size), *image_sub, *image_info_sub,
			*pointcloud_sub));

	sync->registerCallback(boost::bind(&Fusion::callback, this, _1, _2, _3));
}

void Fusion::init_transforms_listener() {
	listener_pointcloud_transform.reset(
			new tf::TransformListener(nh, ros::Duration(config_.tf_buffer_length),
					true));
}

void Fusion::init_pub() {
	pub_cloud_ = nh.advertise<PointCloudColor>(config_.publish_topic_pcl.c_str(), 1);
	it_.reset(new image_transport::ImageTransport(nh));
	pub_ = it_->advertise(config_.publish_topic_image.c_str(), 1);
}

void
Fusion::onInit() {
	NODELET_DEBUG("Initializing nodelet...");

	nh = getPrivateNodeHandle();
	get_param();
	// Info
	print();
	init_sub();
	init_transforms_listener();
	init_pub();
	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Fusion::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}


void
Fusion::callback(const sensor_msgs::ImageConstPtr& input_msg_image, const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	//NODELET_INFO("fusion: callback");
	//time("fusion: ", input_msg_image_info->header, input_msg_cloud_ptr->header, true);

	if(pub_cloud_.getNumSubscribers() == 0 && pub_.getNumSubscribers() == 0 ){ // dont do anything if no one is interessted
		return;
	}

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		NODELET_WARN("input cloud empty");
		return;
	}
	if(input_msg_image->height == 0 || input_msg_image->width == 0){
		NODELET_WARN("input image empty");
		return;
	}

	//Look up transform for cameraladybug_camera
	if(config_.image_tf_frame_id.empty()){ //set default image_frame_id
		config_.image_tf_frame_id = input_msg_image->header.frame_id;
	}

	if(config_.reference_frame.empty()){ //set default reference frame id
		config_.reference_frame = config_.image_tf_frame_id;
    }

	std::string tf_error;
	if(!listener_pointcloud_transform->waitForTransform(config_.image_tf_frame_id.c_str(), //target frame
														input_msg_cloud_ptr->header.frame_id.c_str(), //source frame
														input_msg_image->header.stamp, // target time
														ros::Duration(5.0),
														ros::Duration(0.01),
														&tf_error
														)
	)
	{
		NODELET_WARN("%s: tf_error %s", node_name_.c_str(), tf_error.c_str());
		return;
	}


	//NODELET_INFO("pointcloud2 w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );
	//NODELET_DEBUG(node_name_,"pointcloud2 w: %i \th: %i \t\ttime: %lu",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp );
	//NODELET_DEBUG("image      w: %i \th: %i \ttime: %i.%i",input_msg_image->width, input_msg_image->height, input_msg_image->header.stamp.sec, input_msg_image->header.stamp.nsec );

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr, cloud);

	// Transform to odom
	// todo: Transform at pcl time to odom
	if (!boost::equals(config_.reference_frame, input_msg_cloud_ptr->header.frame_id)){
		if (!pcl_ros::transformPointCloud(config_.reference_frame, cloud, cloud, *listener_pointcloud_transform)) {
				NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", config_.reference_frame.c_str());
			 return;
		}
    }

	// Todo: check if Transform at image time to image_frame_id is right
	cloud.header.stamp = input_msg_image->header.stamp.toNSec();

	// Transform to image_frame_id
	if (!pcl_ros::transformPointCloud(config_.image_tf_frame_id.c_str(), cloud, cloud, *listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", config_.image_tf_frame_id.c_str());
		 return;
	}

	// If we are here we can start
	camera_model.fromCameraInfo(input_msg_image_info);

	//NODELET_INFO("camera_model:  tx: %f, ty: %f, cx: %f, cy: %f, fx: %f, fy: %f ", camera_model.Tx(), camera_model.Ty(), camera_model.cx(), camera_model.cy(), camera_model.fx(), camera_model.fy() );
   cv_bridge::CvImagePtr cv_ptr;
   cv_bridge::CvImagePtr cv_shared_ptr;
   try{
	   cv_ptr = cv_bridge::toCvCopy(input_msg_image, input_msg_image->encoding);
	   cv_shared_ptr = cv_bridge::toCvCopy(input_msg_image, input_msg_image->encoding);
   }
   catch (cv_bridge::Exception& e)
   {
	   ROS_ERROR("cv_bridge exception: %s", e.what());
	   return;
   }

	PointCloudColor::Ptr msg (new PointCloudColor);


	cv::Point2d point_image;
	cv::Vec3b color;

	try{

	   BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points) {
		   if( pt.z > 1){ // min distance from camera 1m
				point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

				// todo: check for view obstructed by robot
				if( ( point_image.x > 0 &&  point_image.x < input_msg_image->width )
					&& ( point_image.y > 0 &&  point_image.y < input_msg_image->height )
					)
				{
					// Get image Color

					color = cv_shared_ptr->image.at<cv::Vec3b>(point_image);
					if( color.val[0] > config_.min_color
						&& color.val[1] > config_.min_color
						&& color.val[2] > config_.min_color
						)
					{
						cv::circle(cv_ptr->image, point_image, 1, cv::Scalar(1,200,1));

						pcl::PointXYZRGB color_point(color.val[2], color.val[1], color.val[0]);
						color_point.x = pt.x;
						color_point.y = pt.y;
						color_point.z = pt.z;
						msg->points.push_back(color_point);
					}
					else{
						cv::circle(cv_ptr->image, point_image, 1, cv::Scalar(1,1,200));
					}
				}
		   }
	   }
	}catch(std::exception &e){
		NODELET_WARN("fusion exception: %s", e.what());
		return;
	}

   msg->header.stamp = cloud.header.stamp;
   msg->header.frame_id = config_.image_tf_frame_id;
   msg->height = 1;
   msg->width = msg->points.size();

   if (!boost::equals(config_.reference_frame, config_.image_tf_frame_id))
   {
	   if (!pcl_ros::transformPointCloud(config_.reference_frame.c_str(), *msg, *msg, *listener_pointcloud_transform)) {
	   			NODELET_WARN("Cannot transform point cloud to the reference frame %s.", config_.reference_frame.c_str());
	   		 return;
	   	}
   }

   pub_cloud_.publish(msg);
   pub_.publish(cv_ptr->toImageMsg());

   NODELET_DEBUG("fusion cloud_color  w: %i \th: %i \ttime: %lu", msg->width, msg->height, msg->header.stamp );
}

void
Fusion::reconfigure_callback(Config &config, uint32_t level) {
	NODELET_INFO( "Fusion: Reconfigure Request");
	bool reset_sub = false;
	bool reset_pub = false;

	if(config.subscribe_topic_pcl != config_.subscribe_topic_pcl
		|| config.subscribe_topic_image != config_.subscribe_topic_image
		|| config.subscribe_topic_image_info != config_.subscribe_topic_image_info
		)
	 {
		reset_sub = true;
	 }

	if(config.publish_topic_image != config_.publish_topic_image
		|| config.publish_topic_pcl != config_.publish_topic_pcl)
	{
		reset_pub = true;
	}

	 config_ = config;
	 print();

	 if(reset_sub){
		 // TODO: Fix boost::lock_error on init_sub() after reconfiguration
		 // Error Message: Reconfigure callback failed with exception boost::lock_error
		 NODELET_WARN("Changing subscriber not implemented, change subscriber in launch file and restart node");
		 //init_sub();
	 }

	 //if( config.tf_buffer_length != config_.tf_buffer_length) init_transforms_listener();

	 if(reset_pub){
		 init_pub();
	 }
}

Fusion::~Fusion(){
	}

} /* end namespace */
