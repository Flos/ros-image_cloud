#include <stdio.h>

#include <sensor_msgs/PointCloud2.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_cloud/calibration/continuous_calibration.h>

namespace image_cloud{
	class Auto_calibration_nodelet: public nodelet::Nodelet
	{

	public:
		Auto_calibration_nodelet(){}
		~Auto_calibration_nodelet(){}
	   virtual void onInit(){
		   inst_.reset(new Continuous_calibration(getNodeHandle(), getPrivateNodeHandle()));
	   }
	   boost::shared_ptr<Continuous_calibration> inst_;

	};
}

PLUGINLIB_DECLARE_CLASS(image_cloud, Auto_calibration_nodelet, image_cloud::Auto_calibration_nodelet, nodelet::Nodelet)
