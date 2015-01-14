#include "sync/sync_image.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Sync_image, image_cloud::Sync_image, nodelet::Nodelet)

namespace image_cloud {

	void
	Sync_image::onInit() {
		NODELET_INFO("Initializing remap_sync...");
		nh_ = getPrivateNodeHandle();
		it_.reset( new image_transport::ImageTransport(nh_));
		init_params();
		init_sub();
		init_pub();

		// Set up dynamic reconfigure
		reconfigure_server_.reset(new ReconfigureServer(nh_));
		ReconfigureServer::CallbackType f = boost::bind(&Sync_image::reconfigure_callback, this, _1, _2);
		reconfigure_server_->setCallback(f);
	}

	void
	Sync_image::init_params(){
		NODELET_INFO("init params remap_sync...");
		nh_.param<std::string>("node_name", node_name_, "Sync_image");

		nh_.param<std::string>("subscribe_topic_image1", 		config_.subscribe_topic_image1, "");
		nh_.param<std::string>("subscribe_topic_image2",  		config_.subscribe_topic_image2, "");
		nh_.param<std::string>("publish_topic_image1",  		config_.publish_topic_image1, "");
		nh_.param<std::string>("publish_topic_image2", 			config_.publish_topic_image2, "");

		nh_.param<int>("queue_size", 							config_.queue_size, 30);

		params();
	}

	void
	Sync_image::params(){
		NODELET_INFO("%s: echo params sync_image....", node_name_.c_str());
		// Info
		NODELET_INFO( "name:\t\t%s", 								node_name_.c_str());
		NODELET_INFO( "subscribe_topic_image1:\t\t%s -> %s",	 	config_.subscribe_topic_image1.c_str(), 	config_.publish_topic_image1.c_str());
		NODELET_INFO( "subscribe_topic_image2:\t\t%s -> %s", 		config_.subscribe_topic_image2.c_str(),		config_.publish_topic_image2.c_str());
		NODELET_INFO( "queue_size:\t\t\t%i",						config_.queue_size);
	}

	void
	Sync_image::init_sub() {
		NODELET_INFO( "remap_sync init_sub");
	//	image_info_sub.reset();
	//	pointcloud_sub.reset();
	//	sync.reset();

		if(config_.subscribe_topic_image1.empty()){ NODELET_ERROR("%s: subscribe_topic_image1 cannot be empty", node_name_.c_str()); return;};
		if(config_.subscribe_topic_image2.empty()){ NODELET_ERROR("%s: subscribe_topic_image2 cannot be empty", node_name_.c_str()); return;};


		image_sub.reset(
				new message_filters::Subscriber<sensor_msgs::Image>(nh_,
						config_.subscribe_topic_image1, config_.queue_size));
		image_sub2.reset(
				new message_filters::Subscriber<sensor_msgs::Image>(nh_,
						config_.subscribe_topic_image2, config_.queue_size));

		// ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
		sync.reset(
				new message_filters::Synchronizer<Image_sync_filter>(
						Image_sync_filter(config_.queue_size),
							*image_sub,
							*image_sub2
							)
				);
		sync->registerCallback(
				boost::bind(&Sync_image::callback, this, _1, _2));
	}

	void
	Sync_image::init_pub() {
		NODELET_INFO("remap_sync init_pub ");
		if(config_.publish_topic_image1.empty()){ NODELET_ERROR("%s: publish_topic_pcl cannot be empty", node_name_.c_str()); return;};
		if(config_.publish_topic_image2.empty()){ NODELET_ERROR("%s: publish_topic_image cannot be empty", node_name_.c_str()); return;};

		pub_image_ 	= 	it_->advertise(config_.publish_topic_image1.c_str(), 1);
		pub_image2_ =  	it_->advertise(config_.publish_topic_image2.c_str(), 1);
	}

	void
	Sync_image::reconfigure_callback(Config &config, uint32_t level){
		NODELET_INFO("%s: sync image remap reconfiguration_callback", node_name_.c_str());
		config_lock_.lock();
		bool reset_sub = false;
		bool reset_pub = false;

		if(config.subscribe_topic_image1 != config_.subscribe_topic_image1
			|| config.subscribe_topic_image2 != config_.subscribe_topic_image2
			)
		 {
			reset_sub = true;
		 }

		if( config.publish_topic_image1 != config_.publish_topic_image1
			|| config.publish_topic_image1 != config_.publish_topic_image1
			){
			reset_pub = true;
		}

		 config_ = config;
		 params();

		 if(reset_sub){
			 // TODO: Fix boost::lock_error on init_sub() after reconfiguration
			 // Error Message: Reconfigure callback failed with exception boost::lock_error
			 NODELET_WARN("%s: Changing subscriber not implemented, change subscriber in launch file and restart node" ,node_name_.c_str());
			 //init_sub();
		 }

		 if(reset_pub){
			 init_pub();
		 }
		 config_lock_.unlock();
	}


	void
	Sync_image::callback(const sensor_msgs::ImageConstPtr& input1, const sensor_msgs::ImageConstPtr& input2){
		config_lock_.lock();

		pub_image_.publish(input1);
		pub_image2_.publish(input2);

		config_lock_.unlock();
	}

	Sync_image::~Sync_image(){
	}

} /* end namespace */
