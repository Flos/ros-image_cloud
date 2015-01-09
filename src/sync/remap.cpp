#include "sync/remap.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Remap, image_cloud::Remap, nodelet::Nodelet)

namespace image_cloud {

	void
	Remap::onInit() {
		NODELET_INFO("Initializing remap_sync...");
		nh_ = getPrivateNodeHandle();
		it_.reset( new image_transport::ImageTransport(nh_));
		init_params();
		init_sub();
		init_pub();

		// Set up dynamic reconfigure
		reconfigure_server_.reset(new ReconfigureServer(nh_));
		ReconfigureServer::CallbackType f = boost::bind(&Remap::reconfigure_callback, this, _1, _2);
		reconfigure_server_->setCallback(f);
	}

	void
	Remap::init_params(){
		NODELET_INFO("init params remap_sync...");
		nh_.param<std::string>("node_name", node_name_, "sync_remap");

		nh_.param<std::string>("subscribe_topic_pcl", 		config_.subscribe_topic_pcl, "");
		nh_.param<std::string>("subscribe_topic_image",  	config_.subscribe_topic_image, "");
		nh_.param<std::string>("subscribe_topic_image_info", config_.subscribe_topic_image_info, "");
		nh_.param<std::string>("publish_topic_pcl", 			config_.publish_topic_pcl, "");
		nh_.param<std::string>("publish_topic_image",  		config_.publish_topic_image, "");
		nh_.param<std::string>("publish_topic_image_info", 	config_.publish_topic_image_info, "");

		nh_.param<bool>("republish_pointcloud", 				config_.republish_pointcloud, 30);
		nh_.param<int>("queue_size", 						config_.queue_size, 30);

		params();
	}

	void
	Remap::params(){
		NODELET_INFO("echo params remap_sync...");
		// Info
		NODELET_INFO( "name:\t\t%s", node_name_.c_str());

		NODELET_INFO( "subscribe_topic_pcl:\t\t%s -> %s",	 		config_.subscribe_topic_pcl.c_str(), 		config_.publish_topic_pcl.c_str());
		NODELET_INFO( "subscribe_topic_image:\t\t%s -> %s", 		config_.subscribe_topic_image.c_str(),		config_.publish_topic_image.c_str());
		NODELET_INFO( "subscribe_topic_image_info: \t%s -> %s", 	config_.subscribe_topic_image_info.c_str(), config_.publish_topic_image_info.c_str());

		NODELET_INFO( "republish_pointcloud: \t%d",		 	config_.republish_pointcloud);
		NODELET_INFO( "queue_size:\t\t\t%i",				config_.queue_size);
	}

	void
	Remap::init_sub() {
		NODELET_INFO( "remap_sync init_sub");
	//	image_info_sub.reset();
	//	pointcloud_sub.reset();
	//	sync.reset();

		if(config_.subscribe_topic_pcl.empty()){ NODELET_ERROR("subscribe_topic_pcl cannot be empty"); return;};
		if(config_.subscribe_topic_image.empty()){ NODELET_ERROR("subscribe_topic_image cannot be empty"); return;};
		if(config_.subscribe_topic_image_info.empty()){ NODELET_ERROR("subscribe_topic_image_info cannot be empty"); return;};

		pointcloud_sub.reset(
				new message_filters::Subscriber<PointCloud>(nh_,
						config_.subscribe_topic_pcl, config_.queue_size));
		image_sub.reset(
				new message_filters::Subscriber<sensor_msgs::Image>(nh_,
						config_.subscribe_topic_image, config_.queue_size));
		image_info_sub.reset(
				new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_,
						config_.subscribe_topic_image_info, config_.queue_size));

		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		sync.reset(
				new message_filters::Synchronizer<Image_to_cloud_sync>(
						Image_to_cloud_sync(config_.queue_size),
							*image_sub,
							*image_info_sub,
							*pointcloud_sub
							)
				);
		sync->registerCallback(
				boost::bind(&Remap::callback, this, _1, _2, _3));
	}

	void
	Remap::init_pub() {
		NODELET_INFO("remap_sync init_pub ");
		if(config_.publish_topic_pcl.empty()){ NODELET_ERROR("publish_topic_pcl cannot be empty"); return;};
		if(config_.publish_topic_image.empty()){ NODELET_ERROR("publish_topic_image cannot be empty"); return;};
		if(config_.publish_topic_image_info.empty()) { NODELET_ERROR("publish_topic_image_info cannot be empty"); return;};


		pub_cloud_ =  nh_.advertise<PointCloud>(config_.publish_topic_pcl.c_str(),1);
		pub_image_ = it_->advertise(config_.publish_topic_image.c_str(), 1);
		pub_image_info_ = nh_.advertise<sensor_msgs::CameraInfo>(config_.publish_topic_image_info.c_str(),1);
	}

	void
	Remap::reconfigure_callback(Config &config, uint32_t level){
		NODELET_INFO("remap_sync reconfiguration_callback");
		config_lock_.lock();
		bool reset_sub = false;
		bool reset_pub = false;

		if(config.subscribe_topic_pcl != config_.subscribe_topic_pcl
			|| config.subscribe_topic_image != config_.subscribe_topic_image
			|| config.subscribe_topic_image_info != config_.subscribe_topic_image_info
			)
		 {
			reset_sub = true;
		 }

		if(config.publish_topic_pcl != config_.publish_topic_pcl
			|| config.publish_topic_image != config_.publish_topic_image
			|| config.publish_topic_image_info != config_.publish_topic_image_info
			){
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
	Remap::callback(const sensor_msgs::ImageConstPtr& input_msg_image, const sensor_msgs::CameraInfoConstPtr &input_msg_image_info, const PointCloud::ConstPtr &input_msg_cloud_ptr){
		config_lock_.lock();

		if(config_.republish_pointcloud) pub_cloud_.publish(input_msg_cloud_ptr);

		pub_image_.publish(input_msg_image);
		pub_image_info_.publish(input_msg_image_info);

		config_lock_.unlock();
	}

	Remap::~Remap(){
	}

} /* end namespace */
