#include "calibration/image_cloud_nodelet.h"
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
	nh.param<std::string>("pub_img", publish_img_topic_, subscribe_topic_pcl_ + "_color");
	nh.param<std::string>("pub_pcl", publish_pcl_topic_, subscribe_topic_img_ + "_color");
	nh.param<std::string>("filter", filter_, "default");
	nh.param<std::string>("image_frame_id", image_frame_id_, "");
	nh.param<std::string>("reference_frame_id", reference_frame_id_, "");
	nh.param<int>("min_color", min_color_val_, 8);
	nh.param<int>("tf_buffer_length", tf_buffer_length_, 8);


	// 2. Info
	NODELET_DEBUG("name:\t\t%s", node_name_.c_str());
	NODELET_DEBUG("sub_pcl: \t%s", subscribe_topic_pcl_.c_str());
	NODELET_DEBUG("sub_img: \t%s", subscribe_topic_img_.c_str());
	NODELET_DEBUG("sub_img_info: \t%s", subscribe_topic_img_info_.c_str());
	NODELET_DEBUG("pub_img:\t\t%s", publish_img_topic_.c_str());
	NODELET_DEBUG("pub_pcl:\t\t%s", publish_pcl_topic_.c_str());
	NODELET_DEBUG("min_color: \t%i", min_color_val_);
	NODELET_DEBUG("tf_buffer_length: \t%i", tf_buffer_length_);

	if(!image_frame_id_.empty()){
		NODELET_INFO("override image frame id: \t%s", image_frame_id_.c_str());
	}

	if(!reference_frame_id_.empty()){
		NODELET_INFO("publish cloud transformed to reference frame: \t%s", reference_frame_id_.c_str());
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

	listener_pointcloud_transform.reset(new tf::TransformListener(nh, ros::Duration(tf_buffer_length_), true));

	pub_cloud_ =  nh.advertise<PointCloudColor>(publish_pcl_topic_.c_str(), 1);

	it_ = new image_transport::ImageTransport(nh);
	pub_ = it_->advertise(publish_img_topic_.c_str(), 1);
}


void
Image_cloud_nodelet::callback(const sensor_msgs::ImageConstPtr& input_msg_image, const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
	ROS_INFO_NAMED(node_name_,"callback");

	if(pub_cloud_.getNumSubscribers() == 0 && pub_.getNumSubscribers() == 0 ){ // dont do anything if no one is interessted
		return;
	}

	if(input_msg_cloud_ptr->height == 0 || input_msg_cloud_ptr->width == 0){
		NODELET_DEBUG("input cloud empty");
		return;
	}
	if(input_msg_image->height == 0 || input_msg_image->width == 0){
		NODELET_DEBUG("input image empty");
		return;
	}


	//Look up transform for cameraladybug_camera4

	if(image_frame_id_.empty()){ //set default image_frame_id
		image_frame_id_ = input_msg_image->header.frame_id;
	}

	if(reference_frame_id_.empty()){ //set default reference frame id
			   reference_frame_id_ = image_frame_id_;
    }

	std::string tf_error;
	if(listener_pointcloud_transform->waitForTransform(image_frame_id_.c_str(), //target frame
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


	ROS_INFO_NAMED(node_name_,"pointcloud2 w: %i \th: %i \t\ttime: %i.%i",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp.sec, input_msg_cloud_ptr->header.stamp.nsec );
	//ROS_INFO_NAMED(node_name_,"pointcloud2 w: %i \th: %i \t\ttime: %lu",input_msg_cloud_ptr->width, input_msg_cloud_ptr->height, input_msg_cloud_ptr->header.stamp );
	ROS_INFO_NAMED(node_name_,"image      w: %i \th: %i \ttime: %i.%i",input_msg_image->width, input_msg_image->height, input_msg_image->header.stamp.sec, input_msg_image->header.stamp.nsec );

	pcl::PointCloud<pcl::PointXYZ> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr,cloud);

	// Transform to odom
	// todo: Transform at pcl time to odom
	if (!pcl_ros::transformPointCloud("/odom", cloud, cloud, *listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", "/odom");
	     return;
	}

	// Todo: check if Transform at image time to image_frame_id is right
	cloud.header.stamp = input_msg_image->header.stamp.toNSec();

	// Transform to image_frame_id
	if (!pcl_ros::transformPointCloud(image_frame_id_.c_str(), cloud, cloud, *listener_pointcloud_transform)) {
			NODELET_WARN("Cannot transform point cloud to the fixed frame %s.", image_frame_id_.c_str());
		 return;
	}

	// If we are here we can start
	camera_model.fromCameraInfo(input_msg_image_info);

	ROS_INFO_NAMED(node_name_,"camera_model:  tx: %f, ty: %f, cx: %f, cy: %f, fx: %f, fy: %f ", camera_model.Tx(), camera_model.Ty(), camera_model.cx(), camera_model.cy(), camera_model.fx(), camera_model.fy() );

	PointCloudColor::Ptr msg (new PointCloudColor);

	cv::Point2d point_image;
	cv::Vec3b color;

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

   int i = 0;
   BOOST_FOREACH (const pcl::PointXYZ& pt, cloud.points) {
		// look up 3D position
		// printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
		// project to 2D Image
	   if( pt.z > 1){ // min distance from camera 1m
			point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0 &&  point_image.x < input_msg_image->width )
				&& ( point_image.y > 0 &&  point_image.y < input_msg_image->height )
				)
			{
				// Get image Color

				color = cv_shared_ptr->image.at<cv::Vec3b>(point_image);
				if( color.val[0] > min_color_val_	// todo replace this with image a image map
					&& color.val[1] > min_color_val_
					&& color.val[2] > min_color_val_
					)
				{
					//debug
					cv::circle(cv_ptr->image, point_image, 1, cv::Scalar(1,200,1));

					//ROS_INFO_NAMED(node_name_,"blend");
					pcl::PointXYZRGB color_point(color.val[2], color.val[1], color.val[0]);
					color_point.x = pt.x;
					color_point.y = pt.y;
					color_point.z = pt.z;
					msg->points.push_back(color_point);
					++i;
				}
				else{
					cv::circle(cv_ptr->image, point_image, 1, cv::Scalar(1,1,200));
				}
				//printf ("image: \t(%f, %f)\n", point_image.x, point_image.y);
			}
			// add colored pixel to cloud_color
	   }
   }

   msg->header.stamp = cloud.header.stamp;
   msg->header.frame_id = image_frame_id_;
   msg->height = 1;
   msg->width = i;


   ROS_INFO_NAMED(node_name_,"1");
   if (!boost::equals(reference_frame_id_, image_frame_id_))
   {
	   ROS_INFO_NAMED(node_name_,"2");

	   if (!pcl_ros::transformPointCloud(reference_frame_id_.c_str(), *msg, *msg, *listener_pointcloud_transform)) {
	   			NODELET_WARN("Cannot transform point cloud to the reference frame %s.", reference_frame_id_.c_str());
	   		 return;
	   	}
	   ROS_INFO_NAMED(node_name_,"3");
   }

   pub_cloud_.publish(msg);
   pub_.publish(cv_ptr->toImageMsg());

   ROS_INFO_NAMED(node_name_,"cloud_color  w: %i \th: %i \ttime: %lu",msg->width, msg->height, msg->header.stamp );
   ROS_INFO_NAMED(node_name_,"callback end");
}

Image_cloud_nodelet::~Image_cloud_nodelet(){
		delete image_sub;
		delete pointcloud_sub;
		delete sync;
	}

} /* end namespace */
