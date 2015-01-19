/*
 * Simplegui.cpp
 *
 *  Created on: 17.01.2015
 *      Author: fnolden
 */

#include "gui/simple_gui.h"
#include "common/project2d.hpp"
#include "common/filter_depth_intensity.hpp"
#include "common/depth_filter.hpp"
#include "common/transform.hpp"

namespace image_cloud {

void callback( int pos, void* data)
{
	Simple_gui* ct = reinterpret_cast<Simple_gui*>(data);
	ct->update_values();
	ct->update_view();
}

void callback_scale( int pos, void* data)
{
	Simple_gui* ct = reinterpret_cast<Simple_gui*>(data);
	ct->update_values();
	ct->recreate_config_gui();
	ct->update_view();
}

Simple_gui::Simple_gui() {
	// TODO Auto-generated constructor stub

}

Simple_gui::~Simple_gui() {
	// TODO Auto-generated destructor stub
}

void Simple_gui::init_filter_data() {
	data_slider.filter_max = 3;
	data_slider.filter = data.filter;
	sprintf(filterNames[0], "intensity");
	sprintf(filterNames[1], "depth");
	sprintf(filterNames[2], "depth_intensity");
	sprintf(filterNames[3], "harris");

	filter_data.resize(4);
	filter_data.at(INTENSITY).resize(0);

	filter_data.at(DEPTH).resize(2);
	filter_data.at(DEPTH).at(0).init("neighbors", 1, 30, false);
	filter_data.at(DEPTH).at(1).init("epsilon", 50, 200, 1, 100);

	filter_data.at(DEPTH_INTENSITY).resize(4);
	filter_data.at(DEPTH_INTENSITY).at(0).init("depth", 30, 200, 1, 100 );
	filter_data.at(DEPTH_INTENSITY).at(1).init("intensity", 50, 200, 1, 100);
	filter_data.at(DEPTH_INTENSITY).at(2).init("neighbors", 1, 30, false);
	filter_data.at(DEPTH_INTENSITY).at(3).init("direction_x?", 0, 1, false);
}

void
Simple_gui::init(){
	//load image
	data.path = "/home/fnolden/Bilder/synced_pcd_images/";
	data.image_file = data.path + "synced_pcd_image_000067.jpg";
	data.pcl_file	= data.path + "synced_pcd_image_1414534811121791.pcd";


	window_name = "manual calibration";
	window_name_control = window_name+" control";

	tf_slider.val_max = 100;
	tf_slider.tx = tf_slider.val_max / 2;
	tf_slider.ty = tf_slider.val_max / 2;
	tf_slider.tz = tf_slider.val_max / 2;
	tf_slider.yaw = tf_slider.val_max / 2;
	tf_slider.roll = tf_slider.val_max / 2;
	tf_slider.pitch = tf_slider.val_max / 2;
	tf_slider.translation_scale = 5;
	tf_slider.rotation_scale = 1;


	data.filter = DEPTH_INTENSITY;


	tf_data.tx = 0;
	tf_data.ty = 0;
	tf_data.tz = 0;
	tf_data.yaw = 0;
	tf_data.roll = 0;
	tf_data.pitch = 0;

	init_filter_data();

	cv::namedWindow(window_name.c_str(), CV_GUI_EXPANDED);
	cv::namedWindow(window_name_control.c_str(), CV_GUI_EXPANDED);
	cv::namedWindow(filterNames[data.filter], CV_GUI_NORMAL);

	create_gui();
	create_gui_filter();

	load_pcl();
	load_image();
	load_projection();
	update_view();
	loop();

}

bool
Simple_gui::load_pcl()
{
	//load pcl
	 cloud_file.reset(new pcl::PointCloud<pcl::PointXYZI>);

	  if (pcl::io::loadPCDFile<pcl::PointXYZI> (data.pcl_file.c_str(), *cloud_file) == -1) //* load the file
	  {
		PCL_ERROR ("Couldn't read file %s \n", data.pcl_file.c_str() );
		return false;
	  }
	  return true;
}

void
Simple_gui::load_image(){
	image_file = cv::imread(data.image_file.c_str());
	image_file.copyTo(image_display);
}

void
Simple_gui::load_projection(){
	//TODO: Load form xml...
	// Camera 4 color_rect_info
	sensor_msgs::CameraInfo info_msg;

	info_msg.K[0] = 230.73746400000002;
	info_msg.K[2] = 635.050064;
	info_msg.K[4] = 230.73746400000002;
	info_msg.K[5] = 532.329328;
	info_msg.K[8] = 1;

	info_msg.P[0] = 507.6224208000001;
	info_msg.P[2] = 639.1101408000002;
	info_msg.P[5] = 507.6224208000001;
	info_msg.P[6] = 547.1245216000002;
	info_msg.P[10] = 1;

	camera_model.fromCameraInfo(info_msg);
}

void Simple_gui::update_tf() {
	float value;
	float scale = tf_slider.translation_scale;
	float scale_rot = tf_slider.rotation_scale;
	float max_half = tf_slider.val_max / 2;
	value = (float) ((tf_slider.tx - max_half)) / (float) (tf_slider.val_max);
	value = value * scale;
	std::cout << "Tx: " << tf_data.tx << " -> " << value << std::endl;
	tf_data.tx = value;
	value = (float) ((tf_slider.ty - max_half)) / (float) (tf_slider.val_max);
	value = value * scale;
	std::cout << "Ty: " << tf_data.ty << " -> " << value << std::endl;
	tf_data.ty = value;
	value = (float) ((tf_slider.tz - max_half)) / (float) (tf_slider.val_max);
	value = value * scale;
	std::cout << "Tz: " << tf_data.tz << " -> " << value << std::endl;
	tf_data.tz = value;
	value = (float) ((tf_slider.roll - max_half)) / (float) (tf_slider.val_max);
	value = value * scale_rot;
	std::cout << "Roll: " << tf_data.roll << " -> " << value << std::endl;
	tf_data.roll = value;
	value = (float) ((tf_slider.pitch - max_half))
			/ (float) (tf_slider.val_max);
	value = value * scale_rot;
	std::cout << "Pitch: " << tf_data.pitch << " -> " << value << std::endl;
	tf_data.pitch = value;
	value = (float) ((tf_slider.yaw - max_half)) / (float) (tf_slider.val_max);
	value = value * scale_rot;
	std::cout << "Yaw: " << tf_data.yaw << " -> " << value << std::endl;
	tf_data.yaw = value;
}

void
Simple_gui::update_values()
{
	update_tf();

	if(data_slider.filter != data.filter){
		cv::destroyWindow(filterNames[data.filter]);
		data.filter = (Filter)data_slider.filter;
		create_gui_filter();
	}
}

void
Simple_gui::loop(){
	while(true){
		cv::waitKey(50);
	}
}

void
Simple_gui::update_view(){
	filter3d();
	cv::imshow(window_name.c_str(), image_display);
}

void
Simple_gui::filter3d(){
	filter_lock.lock();
	printf("filter3d %d %d\n", data.filter, DEPTH);
	pcl::PointCloud<pcl::PointXYZI> transformed = *cloud_file;

	//Transform
	transform_pointcloud<pcl::PointXYZI>(transformed, tf_data.tx, tf_data.ty, tf_data.tz, tf_data.roll, tf_data.pitch, tf_data.yaw);
	pcl::PointCloud<pcl::PointXYZI> filtred;

	switch (data.filter)
	{
		case INTENSITY:
			break;
		case DEPTH:
				filter_depth_discontinuity(transformed, filtred,
						filter_data[data.filter][0].value, // neighbors
						filter_data[data.filter][1].get_value()); // epsilon
			break;
		case DEPTH_INTENSITY:
			{
				std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(image_display.rows, std::vector<boost::shared_ptr<pcl::PointXYZI> > (image_display.cols));
				printf("vector size: %lu, %lu\n", map.size(),map[0].size());
				project2d::project_2d(camera_model, transformed, map, image_display.rows, image_display.cols);
				filter::filter_depth_intensity(map, filtred,
						filter_data[data.filter][0].get_value(), // depth
						filter_data[data.filter][1].get_value(), // intensity
						filter_data[data.filter][2].get_value(),	// neighbors
						filter_data[data.filter][3].get_value()); // search direction_x?
				//filter::filter_depth_intensity();
				//std::vector<std::vector<PointT*> > vec;
			}
			break;
		case HARRIS_3D:
			break;
		default:
			break;
	}

	printf("filter %d in: %lu out: %lu\n", (int)data.filter, cloud_file->size(), filtred.size());
	// pointcloud to image
	image_file.copyTo(image_display); //Reset image
	project2d::project_2d(camera_model, filtred, image_display, project2d::INTENSITY);
	filter_lock.unlock();
}

void Simple_gui::create_gui_filter() {
	cv::namedWindow(filterNames[data.filter], CV_GUI_NORMAL);

	for(int i = 0; i < filter_data[data.filter].size(); ++i){
		Filter_value* fv = &filter_data[data.filter][i];
		cv::createTrackbar( fv->name, 				filterNames[data.filter], &fv->value, 		fv->max, &callback, this );
		if(fv->is_float){
			cv::createTrackbar( fv->name+" numerator", 	filterNames[data.filter], &fv->numerator, 	fv->max, &callback, this );
			cv::createTrackbar( fv->name+" denominator", filterNames[data.filter],  &fv->denominator, 	fv->max, &callback, this );
		}
	}
}

void Simple_gui::recreate_config_gui(){
	cv::destroyWindow(filterNames[data.filter]);
	cv::destroyWindow(window_name_control.c_str());
	create_gui_filter();
	create_gui();
}

void
Simple_gui::create_gui(){
	printf("gui\n");
	 char TrackbarName[9][50];
	 sprintf( TrackbarName[0], "Tx (val - %d) * scale cm", tf_slider.val_max/2 );
	 sprintf( TrackbarName[1], "Ty (val - %d) * scale cm", tf_slider.val_max/2 );
	 sprintf( TrackbarName[2], "Tz (val - %d) * scale cm", tf_slider.val_max/2 );
	 sprintf( TrackbarName[3], "Roll val/%d", tf_slider.val_max );
	 sprintf( TrackbarName[4], "Pich val/%d", tf_slider.val_max );
	 sprintf( TrackbarName[5], "Yaw val/%d", tf_slider.val_max );
	 sprintf( TrackbarName[6], "Filter");
	 sprintf( TrackbarName[7], "T scale val/%d", tf_slider.val_max );
	 sprintf( TrackbarName[8], "R scale val/%d", tf_slider.val_max );


	 cv::createTrackbar( TrackbarName[7], window_name_control.c_str(), &tf_slider.translation_scale, tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[0], window_name_control.c_str(), &tf_slider.tx, 		tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[1], window_name_control.c_str(), &tf_slider.ty, 		tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[2], window_name_control.c_str(), &tf_slider.tz, 		tf_slider.val_max, &callback, this );

	 cv::createTrackbar( TrackbarName[8], window_name_control.c_str(), &tf_slider.rotation_scale, 	tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[3], window_name_control.c_str(), &tf_slider.roll, 	tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[4], window_name_control.c_str(), &tf_slider.pitch, 	tf_slider.val_max, &callback, this );
	 cv::createTrackbar( TrackbarName[5], window_name_control.c_str(), &tf_slider.yaw, 	tf_slider.val_max, &callback, this );

	 cv::createTrackbar( TrackbarName[6], window_name_control.c_str(), &data_slider.filter, data_slider.filter_max, &callback, this );
}


} /* namespace image_cloud */
