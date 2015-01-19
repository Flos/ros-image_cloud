#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "gui/simple_gui.h"


int main(){
	printf("Starting \n");
	image_cloud::Simple_gui gui;
	gui.init();



//	  std::cout << "Loaded "
//	            << cloud->width * cloud->height
//	            << " data points from test_pcd.pcd with the following fields: "
//	            << std::endl;
//	  for (size_t i = 0; i < cloud->points.size (); ++i)
//	    std::cout << "    " << cloud->points[i].x
//	              << " "    << cloud->points[i].y
//	              << " "    << cloud->points[i].z
//				  << " "    << cloud->points[i].intensity << std::endl;


	return 0;
}
