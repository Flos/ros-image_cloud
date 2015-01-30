
#ifndef IMAGE_CLOUD_TRANSFORM_PUBLISHER
#define IMAGE_CLOUD_TRANSFROM_PUBLISHER

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


namespace image_cloud{

// Class to publish later the configuration of floating joints
class Transform_publisher{
public:
	Transform_publisher(){
		init();
	}

	~Transform_publisher(){
	}

	void init(){
		tf_broadcaster.reset( new tf::TransformBroadcaster());
	}

	void publish(tf::StampedTransform tf_stamped){
		tf_broadcaster->sendTransform(tf_stamped);
	}

	void publish(tf::Transform tf, std::string parent, std::string child, ros::Time time = ros::Time::now() ){
		tf::StampedTransform tf_stamped(tf, time, parent, child);

		publish(tf_stamped);
	}

	boost::shared_ptr<tf::TransformBroadcaster> tf_broadcaster;
};

}

#endif
