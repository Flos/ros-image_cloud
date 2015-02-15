/*
 * Simplegui.cpp
 *
 *  Created on: 17.01.2015
 *      Author: fnolden
 */

#include <image_cloud/gui/gui_opencv.h>

#include <image_cloud/common/project2d.hpp>
#include <image_cloud/common/filter/pcl/filter_depth_intensity.hpp>
#include <image_cloud/common/filter/pcl/segmentation.hpp>
#include <image_cloud/common/filter/pcl/depth_filter.hpp>
#include <image_cloud/common/filter/pcl/depth_edge.hpp>
#include <image_cloud/common/filter/pcl/normal_diff_filter.hpp>
#include <image_cloud/common/filter/pcl/range_borders.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_radius.hpp>
#include <image_cloud/common/filter/pcl/depth_filter_neighbors.hpp>
#include <image_cloud/common/transform.hpp>
#include <image_cloud/common/calibration/score.hpp>
#include <image_cloud/common/filter/cv/inverse_distance_transform.hpp>
#include <image_cloud/common/filter/cv/edge.hpp>
#include <image_cloud/common/calibration/multi_score.hpp>


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


void Gui_opencv::init_datasets() {
	for (int i = 0; i < config_files.size(); ++i)
	{
		kitti::Dataset city(config_files.at(i));
		Dataset_config city_conf;
		datasets.list_config.push_back(city_conf);
		datasets.list_datasets.push_back(city);
		datasets.list_config.at(i).pos_image.init("image nr", 0,
				datasets.list_datasets.at(i).camera_file_list.at(0).list.size()
						- 1);
		datasets.list_config.at(i).pos_camera.init("camera nr", 0,
				datasets.list_datasets.at(i).camera_list.cameras.size() - 1);
	}
}

void
Gui_opencv::init(){
	//load default values;
	init_tf();
	init_menu_options();

	//load datasets
	config_files.push_back("/media/Daten/kitti/config_barney_0001.txt");
	config_files.push_back("/media/Daten/kitti/config_kitti_0005.txt");
	config_files.push_back("/media/Daten/kitti/config_kitti_0048.txt");
	config_files.push_back("/media/Daten/kitti/hit_0006.txt");
	config_files.push_back("/media/Daten/kitti/hit_0006_mod.txt");
	config_files.push_back("/media/Daten/kitti/barney/graz/0001.txt");
	config_files.push_back("/media/Daten/kitti/barney/graz/0001_add.txt");

	init_datasets();

	window_names.resize(4);
	window_names[window_name::MAIN] = "selector";
	window_names[window_name::IMAGE] = "image";
	window_names[window_name::TRANSFORM] = "manual tf";
	window_names[window_name::CONFIG] = "settings";

	// Default slider values
	datasets.pos_dataset.init("set", 1, datasets.list_datasets.size() -1);
	datasets.processed_image_selector.init("processed image", image_filter::IMAGE_FULL, images.size() -1);
	datasets.filter2d.blur.init("blur filter", image_filter::blur::OFF, filter2d_blur_names.size() -1);
	datasets.filter2d.edge.init("edge filter", image_filter::edge::MAX, filter2d_edge_names.size() -1);
	datasets.projection.init("0 = intensity | depth = 1", 0, 1);
	datasets.pcl_filter.init("pcl filter", pcl_filter::DEPTH_NEIGHBORS ,filter3d_names.size() -1);

	load_pcl();
	load_image();
	load_projection();



	create_gui_filter2d();
	create_gui_manual_tf();
	create_gui_filter3d();
	create_static_gui();

	create_gui_general_conf();
	cv::namedWindow(window_names.at(window_name::IMAGE).c_str(), CV_GUI_EXPANDED);

	update_view();
	loop();

}

void Gui_opencv::init_menu_options() {
	images.resize(7);

	filter3d_names.push_back("off");
	filter3d_names.push_back("depth");
	filter3d_names.push_back("depth_intensity");
	filter3d_names.push_back("depth_edge");
	filter3d_names.push_back("normal_diff");
	filter3d_names.push_back("range_borders");
	filter3d_names.push_back("depth_radius");
	filter3d_names.push_back("other");

	datasets.filter3d_data.resize(filter3d_names.size());
	datasets.filter3d_data.at(pcl_filter::OFF);

	datasets.filter3d_data.at(pcl_filter::DEPTH).resize(2);
	datasets.filter3d_data.at(pcl_filter::DEPTH).at(0).init("neighbors", 2, 30, false);
	datasets.filter3d_data.at(pcl_filter::DEPTH).at(1).init("epsilon", 50, 200, 1, 100);

	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).resize(4);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(0).init("depth", 30, 200, 1, 100 );
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(1).init("intensity", 50, 200, 1, 100);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(2).init("neighbors", 2, 30, false);
	datasets.filter3d_data.at(pcl_filter::DEPTH_INTENSITY).at(3).init("direction y|x", 1, 1, false);

	datasets.filter3d_data.at(pcl_filter::DEPTH_EDGE).resize(2);
	datasets.filter3d_data.at(pcl_filter::DEPTH_EDGE).at(0).init("depth", 30, 100, 1, 100, false, true);
	datasets.filter3d_data.at(pcl_filter::DEPTH_EDGE).at(1).init("neighbors", 1, 30, false );

	datasets.filter3d_data.at(pcl_filter::NORMAL_DIFF).resize(3);
	datasets.filter3d_data.at(pcl_filter::NORMAL_DIFF).at(0).init("scale1", 10, 200, 1, 100 );
	datasets.filter3d_data.at(pcl_filter::NORMAL_DIFF).at(1).init("scale2", 20, 200, 1, 100);
	datasets.filter3d_data.at(pcl_filter::NORMAL_DIFF).at(2).init("threshold", 20, 200, 1, 100);

	datasets.filter3d_data.at(pcl_filter::DEPTH_RADIUS).resize(4);
	datasets.filter3d_data.at(pcl_filter::DEPTH_RADIUS).at(0).init("radius", 30, 200, 1, 100 );
	datasets.filter3d_data.at(pcl_filter::DEPTH_RADIUS).at(1).init("epsilon", 50, 200, 1, 100);
	datasets.filter3d_data.at(pcl_filter::DEPTH_RADIUS).at(2).init("max distance", 20, 200, 1, 1);
	datasets.filter3d_data.at(pcl_filter::DEPTH_RADIUS).at(3).init("min neighbors", 2, 50, false);

	datasets.filter3d_data.at(pcl_filter::DEPTH_NEIGHBORS).resize(3);
	datasets.filter3d_data.at(pcl_filter::DEPTH_NEIGHBORS).at(0).init("neighbors", 8, 50, false);
	datasets.filter3d_data.at(pcl_filter::DEPTH_NEIGHBORS).at(1).init("epsilon", 50, 200, 1, 100);
	datasets.filter3d_data.at(pcl_filter::DEPTH_NEIGHBORS).at(2).init("max distance", 20, 200, 1, 1);


	datasets.filter3d_data.at(pcl_filter::RANGE_BORDERS).resize(1);
	datasets.filter3d_data.at(pcl_filter::RANGE_BORDERS).at(0).init("angular_resolution_deg", 4, 200, 1, 100, false, true );

	filter2d_blur_names.push_back("off");
	filter2d_blur_names.push_back("bilateral");
	filter2d_blur_names.push_back("blur");
	filter2d_blur_names.push_back("gaussian");
	filter2d_blur_names.push_back("median");

	datasets.filter2d.blur_values.resize(filter2d_blur_names.size());
	datasets.filter2d.blur_values.at(image_filter::blur::BILATERAL).push_back(Slider("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::GAUSSIAN).push_back(Slider("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::MEDIAN).push_back(Slider("kernel", 2, 40));
	datasets.filter2d.blur_values.at(image_filter::blur::BLUR).push_back(Slider("kernel", 2, 40));

	filter2d_edge_names.push_back("off");
	filter2d_edge_names.push_back("canny"); //4
	filter2d_edge_names.push_back("laplace"); //3
	filter2d_edge_names.push_back("max");

	datasets.filter2d.edge_values.resize(filter2d_edge_names.size());
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Slider("threashold1", 30, 500, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Slider("threashold2", 100, 500, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Slider("apertureSize", 3, 30));
	datasets.filter2d.edge_values.at(image_filter::edge::CANNY).push_back(Slider("l2gradient?", 0, 1));

	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Slider("kernel", 3, 30));
	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Slider("scale", 1, 30, 1, 1, false, true));
	datasets.filter2d.edge_values.at(image_filter::edge::LAPLACE).push_back(Slider("delta", 0, 30, 1, 1, false, true));

	assert(datasets.filter2d.blur_values.size() == 5 );
	assert(datasets.filter2d.blur_values.at(0).size() == 0);
	assert(datasets.filter2d.blur_values.at(1).size() == 1);
	assert(datasets.filter2d.blur_values.at(2).size() == 1);
	assert(datasets.filter2d.blur_values.at(3).size() == 1);
	assert(datasets.filter2d.blur_values.at(4).size() == 1);

	assert(datasets.filter2d.edge_values.size() == 4 );
	assert(datasets.filter2d.edge_values.at(0).size() == 0);
	assert(datasets.filter2d.edge_values.at(1).size() == 4);
	assert(datasets.filter2d.edge_values.at(2).size() == 3);
	assert(datasets.filter2d.edge_values.at(3).size() == 0);
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
	datasets.list_datasets.at(datasets.pos_dataset.value).pointcloud_file_list.get_fullname(
			filename,
			datasets.list_config.at(datasets.pos_dataset.value).pos_image.value );
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
	int camera = datasets.list_config.at(datasets.pos_dataset.value).pos_camera.value;
	datasets.list_datasets.at(datasets.pos_dataset.value).camera_file_list
			.at(camera).get_fullname(
				filename,
				datasets.list_config.at(datasets.pos_dataset.value).pos_image.value
				);

	images[image_filter::FILE_READ] = cv::imread(filename);

	images[image_filter::IMAGE_EDGE].create(images[image_filter::FILE_READ].rows,
												images[image_filter::FILE_READ].cols,
												CV_8U);
	images[image_filter::IMAGE_GREY].create(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													CV_8U);
	images[image_filter::IMAGE_INVERSE_TRANSFORMED].create(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													CV_8U);
	images[image_filter::IMAGE_POINTS].create(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													CV_8U);
	images[image_filter::IMAGE_BLUR].create(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													CV_8U);
}

void
Gui_opencv::load_projection(){
	sensor_msgs::CameraInfo info_msg;

	int camera = datasets.list_config.at(datasets.pos_dataset.value).pos_camera.value;

	datasets.list_datasets.at(datasets.pos_dataset.value).camera_list.cameras.at(camera).get_camera_info(info_msg);

	camera_model.fromCameraInfo(info_msg);
}


void
Gui_opencv::update_values()
{
	if(datasets.pos_dataset.loaded != datasets.pos_dataset.value){
		load_projection();
		load_image();
		load_pcl();
		//cv::destroyWindow(filter3d_names[datasets.pcl_filter.value]);
		cv::destroyWindow(window_names.at(window_name::CONFIG));
		//cv::destroyWindow(window_names.at(window_name::TRANSFORM));
		//cv::destroyWindow(datasets.filter2d.window_name);
		create_gui_filter3d();
		//create_gui_manual_tf();
		create_gui_general_conf();
		//create_gui_filter2d();
		datasets.pos_dataset.loaded = datasets.pos_dataset.value;
	}
	if(datasets.filter2d.blur.value != datasets.filter2d.blur.loaded
			|| datasets.filter2d.edge.value != datasets.filter2d.edge.loaded
			){
			cv::destroyWindow(datasets.filter2d.window_name);
			create_gui_filter2d();
			datasets.filter2d.blur.loaded = datasets.filter2d.blur.value;
			datasets.filter2d.edge.loaded = datasets.filter2d.edge.value;
		}

	if(datasets.pcl_filter.value != datasets.pcl_filter.loaded){
		cv::destroyWindow(filter3d_names[datasets.pcl_filter.loaded]);
		datasets.pcl_filter.loaded = datasets.pcl_filter.value;
		create_gui_filter3d();
	}

	// Stored per dataset
	if(datasets.list_config.at(datasets.pos_dataset.value).pos_image.value != datasets.list_config.at(datasets.pos_dataset.value).pos_image.loaded){
		load_image();
		load_pcl();
		datasets.list_config.at(datasets.pos_dataset.value).pos_image.loaded = datasets.list_config.at(datasets.pos_dataset.value).pos_image.value;
	}

	if(datasets.list_config.at(datasets.pos_dataset.value).pos_camera.value != datasets.list_config.at(datasets.pos_dataset.value).pos_camera.loaded){
		load_image();
		load_pcl();
		load_projection();
		datasets.list_config.at(datasets.pos_dataset.value).pos_camera.loaded = datasets.list_config.at(datasets.pos_dataset.value).pos_camera.value;
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
	cv::imshow(window_names.at(window_name::IMAGE).c_str(), images[datasets.processed_image_selector.value]);
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
	switch(datasets.filter2d.blur.value){
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

	switch (datasets.filter2d.edge.value)
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
		case image_filter::edge::MAX:
			filter_2d::edge_max<uchar>( images[image_filter::IMAGE_BLUR], images[image_filter::IMAGE_EDGE]);
			break;
	}

	images[image_filter::IMAGE_EDGE].copyTo(images[image_filter::IMAGE_INVERSE_TRANSFORMED]);
	filter_2d::inverse_distance_transformation<uchar,uchar>(images[image_filter::IMAGE_EDGE], images[image_filter::IMAGE_INVERSE_TRANSFORMED]);
	printf("7");

	}catch(cv::Exception &e){
			printf("Filter2d error: %s", e.what());
	}

	filter_lock.unlock();
}

void
Gui_opencv::filter3d(){
	filter_lock.lock();

	printf("filter3d %d %d\n", datasets.pcl_filter.value, pcl_filter::DEPTH);
	pcl::PointCloud<pcl::PointXYZI> transformed = *cloud_file;

	// Transforms velo_cam0
	tf::Transform velo_to_cam0;
	datasets.list_datasets.at(datasets.pos_dataset.value).velodyne_to_cam0.get_transform(velo_to_cam0);

	// Transform cam0_to_cam
	tf::Transform cam0_to_cam;
	int camera = datasets.list_config.at(datasets.pos_dataset.value).pos_camera.value;

	datasets.list_datasets.at(datasets.pos_dataset.value).camera_list.cameras.at(camera).tf_rect.get_transform(cam0_to_cam);

	tf::Transform result = (cam0_to_cam*velo_to_cam0);
	transform_pointcloud(transformed, result);

	// Transform Manual
	transform_pointcloud<pcl::PointXYZI>(transformed, 	tf_data[0].get_value(), tf_data[1].get_value(), tf_data[2].get_value(),
														tf_data[3].get_value(), tf_data[4].get_value(), tf_data[5].get_value());
	pcl::PointCloud<pcl::PointXYZI> filtred;

	std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > map(images[image_filter::FILE_READ].cols,
						std::vector<boost::shared_ptr<pcl::PointXYZI> > (images[image_filter::FILE_READ].rows));

	printf("vector size: %lu, %lu\n", map.size(),map[0].size());

	Projected_pointcloud<pcl::PointXYZI> projected_pointclouds;
	project2d::project_2d(camera_model, transformed, map, projected_pointclouds, images[image_filter::FILE_READ].cols, images[image_filter::FILE_READ].rows);

	switch (datasets.pcl_filter.value)
	{
		case pcl_filter::OFF:
				filtred = transformed;
			break;
		case pcl_filter::DEPTH:
				filter_depth_discontinuity(transformed, filtred,
						datasets.filter3d_data[datasets.pcl_filter.value][0].value, // neighbors
						datasets.filter3d_data[datasets.pcl_filter.value][1].get_value()); // epsilon
			break;
		case pcl_filter::DEPTH_INTENSITY:
					{
						filter::filter_depth_intensity(map, filtred,
								datasets.filter3d_data[datasets.pcl_filter.value][0].get_value(), // depth
								datasets.filter3d_data[datasets.pcl_filter.value][1].get_value(), // intensity
								datasets.filter3d_data[datasets.pcl_filter.value][2].value, // neighbors
								datasets.filter3d_data[datasets.pcl_filter.value][3].get_value()); // search direction_x?
					}
			break;
		case pcl_filter::DEPTH_EDGE:
							{
								filter_3d::depth_edge(map, filtred,
										datasets.filter3d_data[pcl_filter::DEPTH_EDGE][1].get_value(), // depth
										datasets.filter3d_data[pcl_filter::DEPTH_EDGE][0].get_value()); // neighbors
							}
			break;
		case pcl_filter::NORMAL_DIFF:
							{
								filter_3d::normal_diff_filter(transformed, filtred,
										datasets.filter3d_data[pcl_filter::NORMAL_DIFF][0].get_value(), // depth
										datasets.filter3d_data[pcl_filter::NORMAL_DIFF][1].get_value(), // neighbors
										datasets.filter3d_data[pcl_filter::NORMAL_DIFF][2].get_value()); // threshold
							}
			break;
		case pcl_filter::RANGE_BORDERS:
							{
								filter_3d::range_borders(transformed, filtred,
										datasets.filter3d_data[pcl_filter::RANGE_BORDERS][0].get_value()); // threshold
							}
			break;
		case pcl_filter::DEPTH_RADIUS:
							{
								filter_3d::depth_discontinuity_radius(transformed, filtred,
										datasets.filter3d_data[pcl_filter::DEPTH_RADIUS][0].get_value(), // Radius
										datasets.filter3d_data[pcl_filter::DEPTH_RADIUS][1].get_value(), // epsilon (tree search)
										datasets.filter3d_data[pcl_filter::DEPTH_RADIUS][2].get_value(), // max distance
										datasets.filter3d_data[pcl_filter::DEPTH_RADIUS][3].get_value()); // min neighbors
							}
			break;
		case pcl_filter::DEPTH_NEIGHBORS:
							{
								filter_3d::depth_filter_neighbors(transformed, filtred,
										datasets.filter3d_data[pcl_filter::DEPTH_NEIGHBORS][0].get_value(), // neighbors
										datasets.filter3d_data[pcl_filter::DEPTH_NEIGHBORS][1].get_value(), // epsilon (tree search)
										datasets.filter3d_data[pcl_filter::DEPTH_NEIGHBORS][2].get_value()); // max distance
							}
			break;
		case pcl_filter::OTHER:
			{
				filter::segmentation(transformed, filtred);
					//filtred = transformed;
			}
			break;
		default:
			break;
	}

	// pointcloud to image
	images[image_filter::IMAGE_EDGE].copyTo(images[image_filter::IMAGE_FULL]); //Reset image

	project2d::project_2d(camera_model, filtred, images[image_filter::IMAGE_FULL], (project2d::Field) datasets.projection.value );

	images[image_filter::IMAGE_POINTS] = cv::Mat(images[image_filter::FILE_READ].rows,
													images[image_filter::FILE_READ].cols,
													CV_8UC3,
													cv::Scalar(127,127,127));

	project2d::project_2d(camera_model, filtred, images[image_filter::IMAGE_POINTS], (project2d::Field) datasets.projection.value);

	long unsigned score = 0;
	score::objective_function<pcl::PointXYZI,uchar>(camera_model, filtred, images[image_filter::IMAGE_INVERSE_TRANSFORMED], score);

	imwrite("objective_function.jpg", images[image_filter::IMAGE_INVERSE_TRANSFORMED]);
	imwrite("projected.jpg", images[image_filter::IMAGE_FULL]);
	imwrite("original.jpg", images[image_filter::FILE_READ]);
	imwrite("edgy.jpg", images[image_filter::IMAGE_EDGE]);
	printf("filter %d in: %lu out: %lu score: %lu \n", (int)datasets.pcl_filter.value, cloud_file->size(), filtred.size(), score);

	filter_lock.unlock();
}

void Gui_opencv::create_gui_filter3d() {
	if(datasets.pcl_filter.value != pcl_filter::OFF)
	{
		cv::namedWindow(filter3d_names[datasets.pcl_filter.value], CV_GUI_NORMAL);

		for(int i = 0; i < datasets.filter3d_data[datasets.pcl_filter.value].size(); ++i){
			datasets.filter3d_data[datasets.pcl_filter.value][i]
					  .create_slider(filter3d_names[datasets.pcl_filter.value], &callback, this);
		}
	}
}

void Gui_opencv::create_gui_filter2d() {
	if(datasets.filter2d.blur.value != image_filter::blur::OFF
			||  datasets.filter2d.edge.value != image_filter::edge::OFF )
	{
		datasets.filter2d.window_name = filter2d_blur_names.at(datasets.filter2d.blur.value) + " " + filter2d_edge_names.at(datasets.filter2d.edge.value);
		cv::namedWindow(datasets.filter2d.window_name, CV_GUI_NORMAL);

		// Blur
		for(int i = 0; i < datasets.filter2d.blur_values.at(datasets.filter2d.blur.value).size(); ++i){
			datasets.filter2d.blur_values.at(datasets.filter2d.blur.value).at(i)
				  .create_slider(datasets.filter2d.window_name, &callback, this);
		}

		// Edge
		for(int i = 0; i < datasets.filter2d.edge_values.at(datasets.filter2d.edge.value).size(); ++i){
			datasets.filter2d.edge_values.at(datasets.filter2d.edge.value).at(i)
				  .create_slider(datasets.filter2d.window_name, &callback, this);
		}
	}
}

void Gui_opencv::recreate_config_gui(){
	cv::destroyWindow(window_names.at(window_name::CONFIG));
	//cv::destroyWindow(filter3d_names[datasets.pcl_filter.value]);
	//cv::destroyWindow(window_names.at(window_name::TRANSFORM).c_str());
	create_gui_filter3d();
	//create_gui_manual_tf();
	create_gui_general_conf();
}

void Gui_opencv::create_gui_manual_tf() {
	cv::namedWindow(window_names.at(window_name::TRANSFORM), CV_GUI_EXPANDED);
	tf_data[0].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
	tf_data[1].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
	tf_data[2].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
	tf_data[3].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
	tf_data[4].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
	tf_data[5].create_slider(window_names.at(window_name::TRANSFORM), &callback, this);
}

void
Gui_opencv::create_static_gui(){
	cv::namedWindow(window_names.at(window_name::MAIN).c_str(), CV_GUI_EXPANDED);
	datasets.pcl_filter.create_slider(window_names.at(window_name::MAIN), &callback, this);
	datasets.filter2d.blur.create_slider(window_names.at(window_name::MAIN), &callback, this);
	datasets.filter2d.edge.create_slider(window_names.at(window_name::MAIN), &callback, this);
	datasets.pos_dataset.create_slider(window_names.at(window_name::MAIN), &callback, this);
}

void
Gui_opencv::create_gui_general_conf(){
	cv::namedWindow(window_names.at(window_name::CONFIG).c_str(), CV_GUI_EXPANDED);
	datasets.list_config.at(datasets.pos_dataset.value).pos_image.create_slider(window_names.at(window_name::CONFIG), &callback, this);
	datasets.list_config.at(datasets.pos_dataset.value).pos_camera.create_slider(window_names.at(window_name::CONFIG), &callback, this);

	datasets.processed_image_selector.create_slider(window_names.at(window_name::CONFIG), &callback_image, this);
	datasets.projection.create_slider(window_names.at(window_name::CONFIG), &callback, this);
}


} /* namespace image_cloud */
