#include "calibration/manual_calibration.h"
#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
//PLUGINLIB_DECLARE_CLASS(image_cloud, Manual_calibration, image_cloud::Manual_calibration, nodelet::Nodelet)

namespace image_cloud {

void
Manual_calibration::onInit() {
	//NODELET_DEBUG("Initializing nodelet...");
	config_.x = 0;
	config_.y = 0;
	config_.z = 0;
	config_.roll = 0;
	config_.pitch = 0;
	config_.yaw = 0;
	nh = ros::NodeHandle("~");
	//tfListener.reset(new tf2_ros::TransformListener(tfBuffer));
	initParams();

//  JointState would be a better solution instead of publishing transforms, but floating links currently don't work properly
//	pub = nh.advertise<sensor_msgs::JointState>(pub_topic_.c_str(), 2);
//	pub.publish(msg_);

	// Set up dynamic reconfigure
	reconfigure_server_.reset(new ReconfigureServer(nh));
	ReconfigureServer::CallbackType f = boost::bind(&Manual_calibration::reconfiguration_callback, this, _1, _2);
	reconfigure_server_->setCallback(f);
}

void
Manual_calibration::initParams(){
	nh.param<std::string>("parent_frame", config_.parent_frame, "");
	nh.param<std::string>("child_frame", config_.child_frame, "");
	nh.param<std::string>("name", node_name_, "joint_calibration");
	//nh.param<std::string>("publish_topic", pub_topic_, "joint_camera_calibration");
	params();
}

void
Manual_calibration::params(){
	// Info
	ROS_INFO_NAMED(node_name_, "name:\t\t%s", node_name_.c_str());
	ROS_INFO_NAMED(node_name_, "parent_frame:\t%s", config_.parent_frame.c_str());
	ROS_INFO_NAMED(node_name_, "child_frame:\t%s", config_.child_frame.c_str());
	//ROS_INFO_NAMED(node_name_, "publish_topic:\t%s", pub_topic_.c_str());

	// Optional
	ROS_INFO_NAMED(node_name_, "x:\t%f", config_.x);
	ROS_INFO_NAMED(node_name_, "y:\t%f", config_.y);
	ROS_INFO_NAMED(node_name_, "z:\t%f", config_.z);
	ROS_INFO_NAMED(node_name_, "roll:\t%f", config_.roll);
	ROS_INFO_NAMED(node_name_, "pitch:\t%f", config_.pitch);
	ROS_INFO_NAMED(node_name_, "yaw:\t%f", config_.yaw);

//	msg_.header.stamp = ros::Time::now();
//	msg_.name.clear(); msg_.position.clear();
//	msg_.name.push_back(config_.joint_name);
//	msg_.name.push_back(config_.joint_name);
//	msg_.name.push_back(config_.joint_name);
//	msg_.name.push_back(config_.joint_name);
//	msg_.name.push_back(config_.joint_name);
//	msg_.name.push_back(config_.joint_name);
//	msg_.position.push_back(config_.x);
//	msg_.position.push_back(config_.y);
//	msg_.position.push_back(config_.z);
//	msg_.position.push_back(config_.roll);
//	msg_.position.push_back(config_.pitch);
//	msg_.position.push_back(config_.yaw);

	transformStamped.header.frame_id = config_.parent_frame;
	transformStamped.child_frame_id = config_.child_frame;
	transformStamped.transform.translation.x = config_.x;
	transformStamped.transform.translation.y = config_.y;
	transformStamped.transform.translation.z = config_.z;
	tf2::Quaternion q;
	q.setRPY(config_.roll, config_.pitch, config_.yaw);
	transformStamped.transform.rotation.x = q.x();
	transformStamped.transform.rotation.y = q.y();
	transformStamped.transform.rotation.z = q.z();
	transformStamped.transform.rotation.w = q.w();
}

void
Manual_calibration::loop(){
	ROS_INFO_NAMED(node_name_, "loop start");
	ros::Rate rate(50);
	while(nh.ok()){
		//geometry_msgs::TransformStamped lookupTransformStamped;
		try{
			//ROS_INFO_NAMED(node_name_, "loop");
			//lookupTransformStamped = tfBuffer.lookupTransform(config_.parent_frame,"ladybug_center", ros::Time(0));
			//ROS_INFO_NAMED(node_name_, "loop");
			transformStamped.header.stamp = ros::Time::now();
			br.sendTransform(transformStamped);
		}catch (tf2::TransformException &ex) {
		      ROS_WARN("%s",ex.what());
		      ros::Duration(1).sleep();
		      continue;
		}

		//pub.publish(msg_);
		ros::spinOnce();
		rate.sleep();
	}
	ROS_WARN_NAMED(node_name_, "loop end");
}

void
Manual_calibration::reconfiguration_callback(Config &config, uint32_t level){
	config_ = config;
	params();
	//pub.publish(msg_);
}

Manual_calibration::~Manual_calibration(){
	}

} /* end namespace */


int main(int argc, char **argv){
	ros::init(argc, argv, "manual_calibration");
	image_cloud::Manual_calibration manual_transform;
	ROS_INFO_NAMED("maunal_calibration", "start");
	manual_transform.onInit();
	manual_transform.loop();
	ROS_INFO_NAMED("maunal_calibration", "done");
}
