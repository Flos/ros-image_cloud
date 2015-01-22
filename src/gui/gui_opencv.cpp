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
#include <common/score.hpp>

namespace image_cloud {

void callback( int pos, void* data)
{
	Gui_opencv* ct = reinterpret_cast<Gui_opencv*>(data);
	ct->update_values();
	ct->update_view();
}

void callback_image(int pos, void* data){
	Gui_opencv* ct = reinterpret_cast<Gui_opencv*>(data);
	ct->update_image();
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

		datasets.list_config.at(i).filter3d = pcl_filter::DEPTH;
		datasets.list_config.at(i).filter3d_selector.val = datasets.list_config.at(i).filter3d;
		datasets.list_config.at(i).filter3d_selector.max = datasets.filter3d_data.size() - 1;

		datasets.list_config.at(i).pos_camera.pos.val = 0;
		datasets.list_config.at(i).pos_camera.pos_loaded = 0;
		datasets.list_config.at(i).pos_camera.pos.max = datasets.list_datasets.at(i).camera_list.cameras.size() - 1;

	}

	datasets.pos_dataset.pos.val = 1;
	datasets.pos_dataset.pos.max = datasets.list_datasets.size() -1;
	datasets.pos_dataset.pos_loaded = datasets.pos_dataset.pos.val;


	window_name = "manual calibration";
	window_name_general_conf = window_name + " image control";

	images.resize(6);

	datasets.processed_image_selector.val = 4;
	datasets.processed_image_selector.max = images.size() -1;

	datasets.projection.val = 0;
	datasets.projection.max = 1;

	init_tf();
	init_filter_data();


	load_pcl();
	load_image();
	load_projection();


	cv::namedWindow(window_name.c_str(), CV_GUI_EXPANDED);


	create_gui_general_conf();
	create_gui_manual_tf();
	create_gui_filter3d();
	create_gui_filter2d();

	update_view();
	loop();

}

void Gui_opencv::init_filter_data() {
	datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.max = 3;
	datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.val = datasets.list_config.at(datasets.pos_dataset.pos_loaded).filter3d;
	sprintf(filter3d_names[0], "off");
	sprintf(filter3d_names[1], "depth");
	sprintf(filter3d_names[2], "depth_intensity");
	sprintf(filter3d_names[3], "harris");

	datasets.filter3d_data.resize(4);
	datasets.filter3d_data.at(pcl_filter::OFF);

	datasets.filter3d_data.at(pcl_filter::DEPTH).resize(2);
	datasets.filter3d_data.at(pcl_filter::DEPTH).at(0).init("neighbors", 2, 30, false);
	datasets.filter3d_data.at(pcl_filter::DEPTH).at(1).init("epsilon", 50, 200, 1, 100);

	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).resize(4);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(0).init("depth", 30, 200, 1, 100 );
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(1).init("intensity", 50, 200, 1, 100);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(2).init("neighbors", 2, 30, false);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(3).init("direction_x?", 0, 1, false);



	filter2d_blur_names.push_back("off");
	filter2d_blur_names.push_back("bilateral");
	filter2d_blur_names.push_back("blur");
	filter2d_blur_names.push_back("gaussian");
	filter2d_blur_names.push_back("median");

	datasets.filter2d.blur.pos.val = 0;
	datasets.filter2d.blur.pos_loaded = datasets.filter2d.blur.pos.val;
	datasets.filter2d.blur.pos.max = filter2d_blur_names.size() -1;

	datasets.filter2d.blur_values.resize(filter2d_blur_names.size());
	datasets.filter2d.blur_values.at(image_filter::blur::BILATERAL).push_back(Filter_value("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::GAUSSIAN).push_back(Filter_value("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::MEDIAN).push_back(Filter_value("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::BLUR).push_back(Filter_value("kernel", 2, 40));

	filter2d_edge_names.push_back("off");
	filter2d_edge_names.push_back("canny"); //4
	filter2d_edge_names.push_back("laplace"); //3

	datasets.filter2d.edge.pos.val = 0;
	datasets.filter2d.edge.pos.max = filter2d_edge_names.size() -1;
	datasets.filter2d.edge.pos_loaded = datasets.filter2d.edge.pos.val;

	datasets.filter2d.edge_values.resize(filter2d_edge_names.size());
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Filter_value("threashold1", 30, 500, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Filter_value("threashold2", 100, 500, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Filter_value("apertureSize", 3, 30));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Filter_value("l2gradient?", 0, 1));

	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Filter_value("kernel", 3, 30));
	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Filter_value("scale", 1, 30, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Filter_value("delta", 0, 30, 1, 1, false, true));

	assert(datasets.filter2d.blur_values.size() == 5 );
	assert(datasets.filter2d.blur_values.at(0).size() == 0);
	assert(datasets.filter2d.blur_values.at(1).size() == 1);
	assert(datasets.filter2d.blur_values.at(2).size() == 1);
	assert(datasets.filter2d.blur_values.at(3).size() == 1);
	assert(datasets.filter2d.blur_values.at(4).size() == 1);

	assert(datasets.filter2d.edge_values.size() == 3 );
	assert(datasets.filter2d.edge_values.at(0).size() == 0);
	assert(datasets.filter2d.edge_values.at(1).size() == 4);
	assert(datasets.filter2d.edge_values.at(2).size() == 3);
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

	images[image_filter::FILE_READ] = cv::imread(filename);
	images[image_filter::IMAGE_POINTS].create(images[image_filter::FILE_READ].rows,
												images[image_filter::FILE_READ].cols,
												images[image_filter::FILE_READ].type());
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
	if(datasets.pos_dataset.pos_loaded != datasets.pos_dataset.pos.val){
		load_projection();
		load_image();
		load_pcl();
		cv::destroyWindow(filter3d_names[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d]);
		cv::destroyWindow(window_name_general_conf);
		cv::destroyWindow(window_name_transform);
		//cv::destroyWindow(datasets.filter2d.window_name);
		create_gui_filter3d();
		create_gui_manual_tf();
		create_gui_general_conf();
		//create_gui_filter2d();
		datasets.pos_dataset.pos_loaded = datasets.pos_dataset.pos.val;
	}
	if(datasets.filter2d.blur.pos.val != datasets.filter2d.blur.pos_loaded
			|| datasets.filter2d.edge.pos.val != datasets.filter2d.edge.pos_loaded
			){
			cv::destroyWindow(datasets.filter2d.window_name);
			create_gui_filter2d();
			datasets.filter2d.blur.pos_loaded = datasets.filter2d.blur.pos.val;
			datasets.filter2d.edge.pos_loaded = datasets.filter2d.edge.pos.val;
		}

	if(datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.val != datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d){
		cv::destroyWindow(filter3d_names[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d]);
		datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d = (pcl_filter::Filter3d)datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.val;
		create_gui_filter3d();
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
	filter2d();
	filter3d();
	update_image();
}

void
Gui_opencv::update_image(){
	cv::imshow(window_name.c_str(), images[datasets.processed_image_selector.val]);
}

void
Gui_opencv::filter2d(){
	filter_lock.lock();

	//images[image_filter::FILE_READ].copyTo(image_2d_filtred);

	if(images[image_filter::FILE_READ].channels() != 1 )
	{
		cvtColor(images[image_filter::FILE_READ], images[image_filter::IMAGE_GREY] , CV_BGR2GRAY );

		assert(images[image_filter::IMAGE_GREY].channels() == 1);
	}
	else{
		images[image_filter::FILE_READ].copyTo(images[image_filter::IMAGE_GREY]);
	}


	try{

	// Pipeline blur;
	switch(datasets.filter2d.blur.pos.val){
		default:
		case image_filter::blur::OFF:
			images[image_filter::IMAGE_GREY].copyTo(images[image_filter::IMAGE_BLUR]);
			break;
		case image_filter::blur::BILATERAL:
			cv::bilateralFilter ( images[image_filter::IMAGE_GREY], images[image_filter::IMAGE_BLUR],
					datasets.filter2d.blur_values.at(image_filter::blur::BILATERAL)[0].value,
					datasets.filter2d.blur_values.at(image_filter::blur::BILATERAL)[0].value*2,
					datasets.filter2d.blur_values.at(image_filter::blur::BILATERAL)[0].value/2);
			break;
	    case image_filter::blur::BLUR:
			cv::blur( images[image_filter::IMAGE_GREY], images[image_filter::IMAGE_BLUR], cv::Size( datasets.filter2d.blur_values.at(image_filter::blur::BLUR)[0].value,
					datasets.filter2d.blur_values.at(image_filter::blur::BLUR)[0].value ), cv::Point(-1,-1) );
			break;
	    case image_filter::blur::GAUSSIAN:
			cv::GaussianBlur( images[image_filter::IMAGE_GREY], images[image_filter::IMAGE_BLUR], cv::Size( datasets.filter2d.blur_values.at(image_filter::blur::GAUSSIAN)[0].value,
					datasets.filter2d.blur_values.at(image_filter::blur::GAUSSIAN)[0].value ), 0, 0 );
			break;
	    case image_filter::blur::MEDIAN:
	    	cv::medianBlur ( images[image_filter::IMAGE_GREY], images[image_filter::IMAGE_BLUR], datasets.filter2d.blur_values.at(image_filter::blur::MEDIAN)[0].value );
	    	break;
	}

	switch (datasets.filter2d.edge.pos.val)
	{
		default:
		case image_filter::edge::OFF:
			images[image_filter::IMAGE_BLUR].copyTo(images[image_filter::IMAGE_EDGE]);
			break;
		case image_filter::edge::CANNY:
			cv::Canny( images[image_filter::IMAGE_BLUR], images[image_filter::IMAGE_EDGE],
					datasets.filter2d.edge_values.at(image_filter::edge::CANNY)[0].get_value(),
					datasets.filter2d.edge_values.at(image_filter::edge::CANNY)[1].get_value(),
					datasets.filter2d.edge_values.at(image_filter::edge::CANNY)[2].value,
					datasets.filter2d.edge_values.at(image_filter::edge::CANNY)[3].value );
			break;
		case image_filter::edge::LAPLACE:
			cv::Laplacian( images[image_filter::IMAGE_BLUR], images[image_filter::IMAGE_EDGE], images[image_filter::IMAGE_BLUR].type(),
					datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE)[0].value,
					datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE)[1].get_value(),
					datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE)[2].get_value() );
			break;
	}

	printf("7");

	//filtred.copyTo(images[image_filter::IMAGE_FULL]);
	}catch(cv::Exception &e){
			printf("Filter2d error: %s", e.what());
	}

	filter_lock.unlock();
}

void
Gui_opencv::filter3d(){
	filter_lock.lock();

	printf("filter3d %d %d\n", datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d, pcl_filter::DEPTH);
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

	std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(images[image_filter::FILE_READ].cols,
						std::vector<boost::shared_ptr<pcl::PointXYZI> > (images[image_filter::FILE_READ].rows));

	printf("vector size: %lu, %lu\n", map.size(),map[0].size());

	project2d::project_2d(camera_model, transformed, map, images[image_filter::FILE_READ].cols, images[image_filter::FILE_READ].rows);

	switch (datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d)
	{
		case pcl_filter::OFF:
			filtred = transformed;
			break;
		case pcl_filter::DEPTH:
				filter_depth_discontinuity(transformed, filtred,
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][0].value, // neighbors
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][1].get_value()); // epsilon
			break;
		case pcl_filter::DEPTH_INTENSITY:
			{
				filter::filter_depth_intensity(map, filtred,
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][0].get_value(), // depth
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][1].get_value(), // intensity
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][2].get_value(), // neighbors
						datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][3].get_value()); // search direction_x?
			}
			break;
		case pcl_filter::HARRIS_3D:
		{

		}
			break;
		default:
			break;
	}

	// pointcloud to image
	images[image_filter::IMAGE_EDGE].copyTo(images[image_filter::IMAGE_FULL]); //Reset image

	project2d::project_2d(camera_model, filtred, images[image_filter::IMAGE_FULL], (project2d::Field) datasets.projection.val );

	images[image_filter::IMAGE_POINTS] = cv::Mat::ones(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													images[image_filter::FILE_READ].type() );
	project2d::project_2d(camera_model, filtred, images[image_filter::IMAGE_POINTS], (project2d::Field) datasets.projection.val);

	float score;
	score::score(map, images[image_filter::IMAGE_EDGE], score);
	printf("filter %d in: %lu out: %lu score: %f\n", (int)datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d, cloud_file->size(), filtred.size(), score);

	filter_lock.unlock();
}

void Gui_opencv::create_gui_filter3d() {
	cv::namedWindow(filter3d_names[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d], CV_GUI_NORMAL);

	for(int i = 0; i < datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d].size(); ++i){
		datasets.filter3d_data[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d][i]
				  .create_slider(filter3d_names[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d], &callback, this);
	}
}

void Gui_opencv::create_gui_filter2d() {
	datasets.filter2d.window_name = filter2d_blur_names.at(datasets.filter2d.blur.pos.val) + " " + filter2d_edge_names.at(datasets.filter2d.edge.pos.val);
	cv::namedWindow(datasets.filter2d.window_name, CV_GUI_NORMAL);

	// Blur
	for(int i = 0; i < datasets.filter2d.blur_values.at(datasets.filter2d.blur.pos.val).size(); ++i){
		datasets.filter2d.blur_values.at(datasets.filter2d.blur.pos.val).at(i)
			  .create_slider(datasets.filter2d.window_name, &callback, this);
	}

	// Edge
	for(int i = 0; i < datasets.filter2d.edge_values.at(datasets.filter2d.edge.pos.val).size(); ++i){
		datasets.filter2d.edge_values.at(datasets.filter2d.edge.pos.val).at(i)
			  .create_slider(datasets.filter2d.window_name, &callback, this);
	}
}

void Gui_opencv::recreate_config_gui(){
	cv::destroyWindow(window_name_general_conf);
	cv::destroyWindow(filter3d_names[datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d]);
	cv::destroyWindow(window_name_transform.c_str());
	create_gui_filter3d();
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

	if( datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.max > 0){
		cv::createTrackbar( "pcl filter", window_name_general_conf, &datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.val,
			datasets.list_config.at(datasets.pos_dataset.pos.val).filter3d_selector.max, &callback, this );
	}

	if( datasets.filter2d.blur.pos.max > 0){
		cv::createTrackbar( "image blur", window_name_general_conf, &datasets.filter2d.blur.pos.val,
			datasets.filter2d.blur.pos.max, &callback, this );
	}

	if( datasets.filter2d.edge.pos.max > 0){
		cv::createTrackbar( "image edge", window_name_general_conf, &datasets.filter2d.edge.pos.val,
			datasets.filter2d.edge.pos.max, &callback, this );
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

	cv::createTrackbar("processed image selector", window_name_general_conf, &datasets.processed_image_selector.val, datasets.processed_image_selector.max, &callback_image, this );
	cv::createTrackbar("Project Depth?", window_name_general_conf, &datasets.projection.val, datasets.projection.max, &callback, this );

}


} /* namespace image_cloud */
