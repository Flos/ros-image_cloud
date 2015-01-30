#include <stdio.h>

#include <sensor_msgs/PointCloud2.h>
#include <image_cloud/calibration/continuous_calibration.h>

int main(int argc, char** argv){
	printf("Starting Auto calibration \n");

	ros::init(argc, argv, "kitti_export");
	ros::NodeHandle node;
	ros::NodeHandle priv_nh("~");


	image_cloud::Continuous_calibration auto_calibration(node, priv_nh);

	ros::Rate rate(60);
	while(node.ok()){
		ros::spinOnce();
		rate.sleep();
	}

	printf("Auto calibration stopped \n");
	return 0;
}
