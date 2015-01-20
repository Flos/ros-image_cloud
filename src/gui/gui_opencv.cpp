/*
 * Simplegui.cpp
 *
 *  Created on: 17.01.2015
 *      Author: fnolden
 */

#include <gui/gui_opencv.h>

#include <common/project2d.hpp>
#include <common/filter_depth_intensity.hpp>
#include <common/depth_filter.hpp>
#include <common/transform.hpp>

namespace image_cloud {

void callback( int pos, void* data)
{
	Gui_opencv* ct = reinterpret_cast<Gui_opencv*>(data);
	ct->update_values();
	ct->update_view();
}

void callback_scale( int pos, void* data)
{
	Gui_opencv* ct = reinterpret_cast<Gui_opencv*>(data);
	ct->update_values();
	ct->recreate_config_gui();
	ct->update_view();
}

Gui_opencv::Gui_opencv() {
	// TODO Auto-generated constructor stub
}

Gui_opencv::~Gui_opencv() {
	// TODO Auto-generated destructor stub
}

void
Gui_opencv::init(){
	//load image
	data.path = "/home/fnolden/Bilder/synced_pcd_images/";
	data.image_file = data.path + "synced_pcd_image_000067.jpg";
	data.pcl_file	= data.path + "synced_pcd_image_1414534811121791.pcd";


	kitti::dataset datas("/home/fnolden/Downloads/2011_09_26_drive_0005_sync/config.txt");

	data.set.images.load_file("/home/fnolden/Downloads/2011_09_26_drive_0005_sync/image_00/files.txt");
	data.set.images.path = "/home/fnolden/Downloads/2011_09_26_drive_0005_sync/image_00/data/";
	data.set.pointclouds.load_file("/home/fnolden/Downloads/2011_09_26_drive_0005_sync/velodyne_points/files.txt");
	data.set.pointclouds.path = "/home/fnolden/Downloads/2011_09_26_drive_0005_sync/velodyne_points/data/";
	data.set.pos_loaded = 0;

	assert (data.set.pointclouds.list.size() == data.set.images.list.size());
	data.set.pos.max = data.set.pointclouds.list.size();
	data.set.pos.val = data.set.pos_loaded;


	window_name = "manual calibration";
	window_name_transform = window_name+" transform";
	window_name_control = window_name + " image control";

	data.filter = DEPTH;

	init_tf();
	init_filter_data();


	load_pcl();
	load_image();
	load_projection();

	cv::namedWindow(window_name.c_str(), CV_GUI_EXPANDED);
	cv::namedWindow(window_name_transform.c_str(), CV_GUI_EXPANDED);
	cv::namedWindow(window_name_control.c_str(), CV_GUI_EXPANDED);
	cv::namedWindow(filterNames[data.filter], CV_GUI_NORMAL);


	create_gui();
	create_gui_filter();

	update_view();
	loop();

}

void Gui_opencv::init_filter_data() {
	data.filter_selector.max = 3;
	data.filter_selector.val = data.filter;
	sprintf(filterNames[0], "intensity");
	sprintf(filterNames[1], "depth");
	sprintf(filterNames[2], "depth_intensity");
	sprintf(filterNames[3], "harris");

	filter_data.resize(4);
	filter_data.at(INTENSITY);

	filter_data.at(DEPTH).resize(2);
	filter_data.at(DEPTH).at(0).init("neighbors", 2, 30, false);
	filter_data.at(DEPTH).at(1).init("epsilon", 50, 200, 1, 100);

	filter_data.at(DEPTH_INTENSITY).resize(4);
	filter_data.at(DEPTH_INTENSITY).at(0).init("depth", 30, 200, 1, 100 );
	filter_data.at(DEPTH_INTENSITY).at(1).init("intensity", 50, 200, 1, 100);
	filter_data.at(DEPTH_INTENSITY).at(2).init("neighbors", 2, 30, false);
	filter_data.at(DEPTH_INTENSITY).at(3).init("direction_x?", 0, 1, false);
}

void Gui_opencv::init_tf(){
	tf_data[0].init("tx", 		50, 100, 1, 10, true, true);
	tf_data[1].init("ty", 		50, 100, 1, 10, true, true);
	tf_data[2].init("tz", 		50, 100, 1, 10, true, true);
	tf_data[3].init("roll", 	50, 100, 1, 10, true, true);
	tf_data[4].init("pitch", 	50, 100, 1, 10, true, true);
	tf_data[5].init("yaw", 		50, 100, 1, 10, true, true);
}

bool
Gui_opencv::load_pcl()
{
	std::string filename;
	data.set.pointclouds.get_fullname(filename, data.set.pos.val);
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
Gui_opencv::load_image(){
	std::string filename;
	data.set.images.get_fullname(filename, data.set.pos.val);
	image_file = cv::imread(filename);
	image_file.copyTo(image_display);
}

void
Gui_opencv::load_projection(){
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

	kitti::Camera cam;

	cam.S[0] = 1;
	cam.S[1] = 0;
	cam.save_file("ldybug_camera4_init.txt");
	cam.set_camera_info(info_msg);
	cam.camera_nr = 4;
	cam.save_file("ldybug_camera4.txt");

	camera_model.fromCameraInfo(info_msg);
}


void
Gui_opencv::update_values()
{
	if(data.filter_selector.val != data.filter){
		cv::destroyWindow(filterNames[data.filter]);
		data.filter = (Filter)data.filter_selector.val;
		create_gui_filter();
	}

	if(data.set.pos.val != data.set.pos_loaded){
		load_image();
		load_pcl();
		data.set.pos_loaded = data.set.pos.val;
	}
}

void
Gui_opencv::loop(){
	while(true){
		cv::waitKey(50);
	}
}

void
Gui_opencv::update_view(){
	filter3d();
	cv::imshow(window_name.c_str(), image_display);
}

void
Gui_opencv::filter3d(){
	filter_lock.lock();
	printf("filter3d %d %d\n", data.filter, DEPTH);
	pcl::PointCloud<pcl::PointXYZI> transformed = *cloud_file;

	//Transform
	transform_pointcloud<pcl::PointXYZI>(transformed, 	tf_data[0].get_value(), tf_data[1].get_value(), tf_data[2].get_value(),
														tf_data[3].get_value(), tf_data[4].get_value(), tf_data[5].get_value());
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
						filter_data[data.filter][2].get_value(), // neighbors
						filter_data[data.filter][3].get_value()); // search direction_x?
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

void Gui_opencv::create_gui_filter() {
	cv::namedWindow(filterNames[data.filter], CV_GUI_NORMAL);

	for(int i = 0; i < filter_data[data.filter].size(); ++i){
		filter_data[data.filter][i].create_slider(filterNames[data.filter], &callback, this);
	}
}

void Gui_opencv::recreate_config_gui(){
	cv::destroyWindow(filterNames[data.filter]);
	cv::destroyWindow(window_name_transform.c_str());
	create_gui_filter();
	create_gui();
}

void
Gui_opencv::create_gui(){
	printf("gui\n");
	tf_data[0].create_slider(window_name_transform, &callback, this);
	tf_data[1].create_slider(window_name_transform, &callback, this);
	tf_data[2].create_slider(window_name_transform, &callback, this);
	tf_data[3].create_slider(window_name_transform, &callback, this);
	tf_data[4].create_slider(window_name_transform, &callback, this);
	tf_data[5].create_slider(window_name_transform, &callback, this);

	cv::createTrackbar( "Filter", window_name_control, &data.filter_selector.val, data.filter_selector.max, &callback, this );
	cv::createTrackbar( "seq", window_name_control, &data.set.pos.val, data.set.pos.max, &callback, this );
}


} /* namespace image_cloud */
