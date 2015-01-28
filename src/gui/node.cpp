#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_cloud/gui/gui_opencv.h>


int main(){
	printf("Starting \n");
	image_cloud::Gui_opencv gui;
	gui.init();

	return 0;
}
