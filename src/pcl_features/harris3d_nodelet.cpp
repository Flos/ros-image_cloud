/*
 * Edgedetectornodelet.cpp
 *
 *  Created on: 29.12.2014
 *      Author: fnolden
 */

#include "pcl_features/harris3d_nodelet.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(image_cloud, Harris3d_nodelet, image_cloud::Harris3d_nodelet, nodelet::Nodelet)

namespace image_cloud {

void
Harris3d_nodelet::onInit() {
	NODELET_DEBUG("Initializing nodelet...");
	nh_ = getPrivateNodeHandle();
	nh_.param<std::string>("name", node_name_, "harris3d");
	nh_.param<std::string>("subscribe_topic", config_.subscribe_topic, "");
	nh_.param<std::string>("publish_topic", config_.publish_topic, config_.subscribe_topic + "_harris3d");

	sub_ = nh_.subscribe(config_.subscribe_topic.c_str(), 1, &Harris3d_nodelet::callback, this);
	pub_ = nh_.advertise<PointCloud>(config_.publish_topic.c_str(), 1);
//
	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh_));
	ReconfigureServer::CallbackType f = boost::bind(&Harris3d_nodelet::reconfigure_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Harris3d_nodelet::callback(const PointCloud::ConstPtr &input_msg_cloud_ptr){
	NODELET_DEBUG("callback");

	if(pub_.getNumSubscribers() == 0) return;

	pcl::PointCloud<pcl::PointXYZI> cloud;
	pcl::fromROSMsg(*input_msg_cloud_ptr,cloud);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree_n(new pcl::search::KdTree<pcl::PointXYZI>());

	// Estimate the normals of the cloud_xyz
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);

	ne.setInputCloud(cloud.makeShared());
	ne.setSearchMethod(tree_n);
	ne.compute(*cloud_normals);

	detector.setInputCloud(cloud.makeShared());
	detector.setNormals(cloud_normals);

	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints(new pcl::PointCloud<pcl::PointXYZI>());
	detector.compute(*keypoints);

	NODELET_INFO( "In: %lu, keypoints: %lu", cloud.size(), keypoints->size());

//	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
//	pcl::PointXYZ tmp;
//	double max = 0,min=0;
//
//	for(pcl::PointCloud<pcl::PointXYZI>::iterator i = keypoints->begin(); i!= keypoints->end(); i++){
//		tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).zne);
//		if ((*i).intensity>max ){
//			std::cout << (*i) << " coords: " << (*i).x << ";" << (*i).y << ";" << (*i).z << std::endl;
//			max = (*i).intensity;
//		}
//		if ((*i).intensity<min){
//			min = (*i).intensity;
//		}
//		keypoints3D->push_back(tmp);
//	}

	//std::cout << "maximal responce: "<< max << " min responce:  "<< min<<std::endl;

	PointCloud out;
	pcl::toROSMsg(*keypoints,out);
	out.header = input_msg_cloud_ptr->header;
	pub_.publish(out);

	NODELET_DEBUG("callback end");
}

void
Harris3d_nodelet::reconfigure_callback(Config &config, uint32_t level) {
	NODELET_INFO( "Reconfigure Request");
	NODELET_INFO( "name:\t%s", node_name_.c_str());
	NODELET_INFO( "subscribe_topic:\t%s", config.subscribe_topic.c_str());
	NODELET_INFO( "publish_topic:\t%s", config.publish_topic.c_str());
	NODELET_INFO( "threshold: \t%f", config.threshold);
	NODELET_INFO( "radius: \t%f", config.radius);
	NODELET_INFO( "radius_search: \t%f", config.radius_search);
	NODELET_INFO( "non_max_supression: \t%s", config.non_max_supression ? "true" : "false");
	NODELET_INFO( "refine: \t%s", config.refine ? "true" : "false");

	detector.setNonMaxSupression(config.non_max_supression);
	detector.setRadius(config.radius);
	detector.setRadiusSearch(config.radius_search);
	detector.setThreshold(config.threshold);
	detector.setRefine(config.refine);

	ne.setRadiusSearch(config_.radius);

	if(config.subscribe_topic != config_.subscribe_topic){
	  sub_ = nh_.subscribe(config.subscribe_topic, 1, &Harris3d_nodelet::callback, this);
	  NODELET_INFO( "Subscribe topic changed from %s to %s", config_.subscribe_topic.c_str(), config.subscribe_topic.c_str());
	  //
	}

	if(config.publish_topic != config_.publish_topic)
	{
	  pub_ = nh_.advertise<PointCloud>(config_.publish_topic.c_str(), 1);
	  NODELET_INFO( "Publish topic changed from %s to %s", config_.publish_topic.c_str(), config.publish_topic.c_str());
	}
	config_ = config;
}

Harris3d_nodelet::Harris3d_nodelet() {
	// TODO Auto-generated constructor stub
}

Harris3d_nodelet::~Harris3d_nodelet() {
	// TODO Auto-generated destructor stub
}

} /* namespace ladybug */
