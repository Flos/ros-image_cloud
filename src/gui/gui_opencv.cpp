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
	config_files.push_back("/media/Daten/kitti/config_barney_0001.txt");
	config_files.push_back("/media/Daten/kitti/config_kitti_0005.txt");
	config_files.push_back("/media/Daten/kitti/config_kitti_0048.txt");

	for(int i = 0; i < config_files.size(); ++i)
	{
		kitti::Dataset city(config_files.at(i));
		Dataset_config city_conf;

		datasets.list_config.push_back(city_conf);
		datasets.list_datasets.push_back(city);

		datasets.list_config.at(i).pos_image.pos.val = 0;
		datasets.list_config.at(i).pos_image.pos.max = 	datasets.list_datasets.at(i).camera_file_list.at(0).list.size() - 1;
		datasets.list_config.at(i).pos_image.pos_loaded = 0;

		datasets.list_config.at(i).pos_camera.pos.val = 0;
		datasets.list_config.at(i).pos_camera.pos.max = datasets.list_datasets.at(i).camera_list.cameras.size() - 1;
		datasets.list_config.at(i).pos_camera.pos_loaded = 0;

		datasets.list_config.at(i).filter = DEPTH;
		datasets.list_config.at(i).filter_selector.val = datasets.list_config.at(i).filter;
		datasets.list_config.at(i).filter_selector.max = datasets.filter_data.size() - 1;

		datasets.list_config.at(i).pos_camera.pos.val = 0;
		datasets.list_config.at(i).pos_camera.pos_loaded = 0;
		datasets.list_config.at(i).pos_camera.pos.max = datasets.list_datasets.at(i).camera_list.cameras.size() - 1;

	}

	datasets.pos_dataset.pos.val = 1;
	datasets.pos_dataset.pos.max = datasets.list_datasets.size() -1;
	datasets.pos_dataset.pos_loaded = 0;


	window_name = "manual calibration";
	window_name_general_conf = window_name + " image control";

	init_tf();
	init_filter_data();


	load_pcl();
	load_image();
	load_projection();


	cv::namedWindow(window_name.c_str(), CV_GUI_EXPANDED);


	create_gui_general_conf();
	create_gui_manual_tf();
	create_gui_filter();

	update_view();
	loop();

}

void Gui_opencv::init_filter_data() {
	datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.max = 3;
	datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.val = datasets.list_config.at(datasets.pos_dataset.pos_loaded).filter;
	sprintf(filterNames[0], "intensity");
	sprintf(filterNames[1], "depth");
	sprintf(filterNames[2], "depth_intensity");
	sprintf(filterNames[3], "harris");

	datasets.filter_data.resize(4);
	datasets.filter_data.at(INTENSITY);

	datasets.filter_data.at(DEPTH).resize(2);
	datasets.filter_data.at(DEPTH).at(0).init("neighbors", 2, 30, false);
	datasets.filter_data.at(DEPTH).at(1).init("epsilon", 50, 200, 1, 100);

	datasets.filter_data.at(DEPTH_INTENSITY).resize(4);
	datasets.filter_data.at(DEPTH_INTENSITY).at(0).init("depth", 30, 200, 1, 100 );
	datasets.filter_data.at(DEPTH_INTENSITY).at(1).init("intensity", 50, 200, 1, 100);
	datasets.filter_data.at(DEPTH_INTENSITY).at(2).init("neighbors", 2, 30, false);
	datasets.filter_data.at(DEPTH_INTENSITY).at(3).init("direction_x?", 0, 1, false);
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
	datasets.list_datasets.at(datasets.pos_dataset.pos.val).pointcloud_file_list.get_fullname(
			filename,
			datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.val );
	//load pcl
	cloud_file.reset(new pcl::PointCloud<pcl::PointXYZI>);

	// Check extension...
	int type = 0;
	if(	 filename[filename.length()-4] == '.'
			&& (filename[filename.length()-3] == 'b'
			|| filename[filename.length()-3] == 'B')
		)
	{

		type = 1;
	}

	switch (type) {
		default:
		case 0:
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZI> (filename.c_str(), *cloud_file) == -1) //* load the file
			{
				PCL_ERROR ("Couldn't read file %s \n", filename.c_str() );
				return false;
			}
			break;
		}
		case 1:
		{
			std::fstream input(filename.c_str(), std::ios::in | std::ios::binary);
			if(!input.good()){
				std::cout << "Could not read file: " << filename << "\n";
			}
			else
			{
				input.seekg(0, std::ios::beg);
				int i;
				for (i=0; input.good() && !input.eof(); i++) {
					pcl::PointXYZI point;
					input.read((char *) &point.x, 3*sizeof(float));
					input.read((char *) &point.intensity, sizeof(float));
					cloud_file->push_back(point);
				}
				input.close();
			}
			break;
		}

	} /* switch end */

	std::cout << "Loaded [" << cloud_file->size() << "] points from file ["<< filename << "]\n";
	return true;
}

void
Gui_opencv::load_image(){
	std::string filename;
	int camera = datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val;
	datasets.list_datasets.at(datasets.pos_dataset.pos.val).camera_file_list
			.at(camera).get_fullname(
				filename,
				datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.val
				);

	image_file = cv::imread(filename);
	image_file.copyTo(image_display);
}

void
Gui_opencv::load_projection(){
	sensor_msgs::CameraInfo info_msg;

	int camera = datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val;

	datasets.list_datasets.at(datasets.pos_dataset.pos.val).camera_list.cameras.at(camera).get_camera_info(info_msg);

	camera_model.fromCameraInfo(info_msg);
}


void
Gui_opencv::update_values()
{
	if(datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.val != datasets.list_config.at(datasets.pos_dataset.pos.val).filter){
		cv::destroyWindow(filterNames[datasets.list_config.at(datasets.pos_dataset.pos.val).filter]);
		datasets.list_config.at(datasets.pos_dataset.pos.val).filter = (Filter)datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.val;
		create_gui_filter();
	}
	if(datasets.pos_dataset.pos_loaded != datasets.pos_dataset.pos.val){
		load_projection();
		load_image();
		load_pcl();
		cv::destroyWindow(filterNames[datasets.list_config.at(datasets.pos_dataset.pos.val).filter]);
		cv::destroyWindow(window_name_general_conf);
		cv::destroyWindow(window_name_transform);
		create_gui_filter();
		create_gui_manual_tf();
		create_gui_general_conf();
		datasets.pos_dataset.pos_loaded = datasets.pos_dataset.pos.val;
	}

	//
	if(datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.val != datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos_loaded){
		load_image();
		load_pcl();
		datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos_loaded = datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.val;
	}

	if(datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val != datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos_loaded){
		load_image();
		load_pcl();
		load_projection();
		datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos_loaded = datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val;
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

	printf("filter3d %d %d\n", datasets.list_config.at(datasets.pos_dataset.pos.val).filter, DEPTH);
	pcl::PointCloud<pcl::PointXYZI> transformed = *cloud_file;

	// Transforms velo_cam0
	tf::Transform velo_to_cam0;
	datasets.list_datasets.at(datasets.pos_dataset.pos.val).velodyne_to_cam0.get_transform(velo_to_cam0);

	// Transform cam0_to_cam
	tf::Transform cam0_to_cam;
	int camera = datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val;

	datasets.list_datasets.at(datasets.pos_dataset.pos.val).camera_list.cameras.at(camera).tf_rect.get_transform(cam0_to_cam);

	tf::Transform result = (velo_to_cam0, velo_to_cam0);
	transform_pointcloud(transformed, result);

	// Transform Manual
	transform_pointcloud<pcl::PointXYZI>(transformed, 	tf_data[0].get_value(), tf_data[1].get_value(), tf_data[2].get_value(),
														tf_data[3].get_value(), tf_data[4].get_value(), tf_data[5].get_value());
	pcl::PointCloud<pcl::PointXYZI> filtred;

	switch (datasets.list_config.at(datasets.pos_dataset.pos.val).filter)
	{
		case INTENSITY:
			filtred = transformed;
			break;
		case DEPTH:
				filter_depth_discontinuity(transformed, filtred,
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][0].value, // neighbors
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][1].get_value()); // epsilon
			break;
		case DEPTH_INTENSITY:
			{
				std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(image_display.rows*3, std::vector<boost::shared_ptr<pcl::PointXYZI> > (image_display.cols*2));
				printf("vector size: %lu, %lu\n", map.size(),map[0].size());
				project2d::project_2d(camera_model, transformed, map, image_display.rows*3, image_display.cols*2);
				filter::filter_depth_intensity(map, filtred,
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][0].get_value(), // depth
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][1].get_value(), // intensity
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][2].get_value(), // neighbors
						datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][3].get_value()); // search direction_x?
			}
			break;
		case HARRIS_3D:
			break;
		default:
			break;
	}

	printf("filter %d in: %lu out: %lu\n", (int)datasets.list_config.at(datasets.pos_dataset.pos.val).filter, cloud_file->size(), filtred.size());
	// pointcloud to image
	image_file.copyTo(image_display); //Reset image
	project2d::project_2d(camera_model, filtred, image_display, project2d::INTENSITY);
	filter_lock.unlock();
}

void Gui_opencv::create_gui_filter() {
	cv::namedWindow(filterNames[datasets.list_config.at(datasets.pos_dataset.pos.val).filter], CV_GUI_NORMAL);

	for(int i = 0; i < datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter].size(); ++i){
		datasets.filter_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter][i]
				  .create_slider(filterNames[datasets.list_config.at(datasets.pos_dataset.pos.val).filter], &callback, this);
	}
}

void Gui_opencv::recreate_config_gui(){
	cv::destroyWindow(filterNames[datasets.list_config.at(datasets.pos_dataset.pos.val).filter]);
	cv::destroyWindow(window_name_transform.c_str());
	create_gui_filter();
	create_gui_manual_tf();
	create_gui_general_conf();
}

void Gui_opencv::create_gui_manual_tf() {
	window_name_transform = window_name+" transform";
	cv::namedWindow(window_name_transform, CV_GUI_EXPANDED);
	tf_data[0].create_slider(window_name_transform, &callback, this);
	tf_data[1].create_slider(window_name_transform, &callback, this);
	tf_data[2].create_slider(window_name_transform, &callback, this);
	tf_data[3].create_slider(window_name_transform, &callback, this);
	tf_data[4].create_slider(window_name_transform, &callback, this);
	tf_data[5].create_slider(window_name_transform, &callback, this);
}

void
Gui_opencv::create_gui_general_conf(){
	cv::namedWindow(window_name_general_conf.c_str(), CV_GUI_EXPANDED);

	if( datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.max > 0){
		cv::createTrackbar( "Filter", window_name_general_conf, &datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.val,
			datasets.list_config.at(datasets.pos_dataset.pos.val).filter_selector.max, &callback, this );
	}

	if( datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.max > 0){
		cv::createTrackbar( "img.nr", window_name_general_conf, &datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.val,
			datasets.list_config.at(datasets.pos_dataset.pos.val).pos_image.pos.max, &callback, this );
	}

	if( datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.max > 0){
		cv::createTrackbar( "camera", window_name_general_conf, &datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.val,
		datasets.list_config.at(datasets.pos_dataset.pos.val).pos_camera.pos.max, &callback, this );
	}

	if(datasets.pos_dataset.pos.max > 0) {
		cv::createTrackbar( "data_set", window_name_general_conf, &datasets.pos_dataset.pos.val, datasets.pos_dataset.pos.max, &callback, this );
	}
}


} /* namespace image_cloud */
