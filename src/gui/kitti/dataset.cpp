/*
 * dataset.cpp
 *
 *  Created on: 20.01.2015
 *      Author: fnolden
 */

#include <gui/kitti/dataset.h>

namespace image_cloud {

namespace kitti {

Dataset::Dataset() {
	valid = false;
}

Dataset::Dataset( std::string config ){
	valid = false;
	init( config );
}

bool
Dataset::init( std::string config ){
	if (! load_config( config)) return false;
	if (! load_cameras()) return false;
	if (! load_camera_files()) return false;
	if (! load_pointcloud_files()) return false;
	if (! load_velodyne_to_cam_tf()) return false;
	return check();
}
Dataset::~Dataset() {
}

void Dataset::extract_string(std::istringstream& in, std::string &file_path) {
	std::istreambuf_iterator<char> it(in), end;
	++it; // remove whitespace
	std::copy(it, end, std::inserter(file_path, file_path.begin()));
}

bool
Dataset::load_config( std::string config){
	path.config_file = config;

	std::cout << "Reading config from: "<< config << '\n';

	std::ifstream stream(path.config_file.c_str());

	if (stream.is_open()) {

		std::string line;

		while ( std::getline (stream, line) )
		{
			std::istringstream in(line);    //make a stream for the line itself

			std::string type;
			in >> type;                  	//read the first whitespace-separated token

			if(type == "camera_calib:")
			{
				extract_string(in, path.camera_calib_file);
				std::cout << type << " " << path.camera_calib_file << "\n";
			}
			if(type == "tf_velo_cam0:")
			{
				extract_string(in, path.tf_velodyne_to_cam0);
				std::cout << type << " " << path.tf_velodyne_to_cam0 << "\n";
			}
			if(type == "data_root:")
			{
				extract_string(in, path.root_data_path);
				std::cout << type << " " << path.root_data_path << "\n";
			}
			if(type == "pcl_data:")
			{
				extract_string(in, path.pcl_data);
				std::cout << type << " " << path.pcl_data << "\n";
			}
			if(type == "camera_data:")
			{
				std::string camera_data;
				extract_string(in, camera_data);
				path.camera_data.list.push_back(camera_data);
				std::cout << type << " " << camera_data << "\n";
			}
		}

		stream.close();
	} else{
		std::cout << "Unable to open file: " << config << "\n";
		return false;
	}
	return true;

}

bool
Dataset::load_cameras(){
	return camera_list.load_file(path.camera_calib_file );
}

bool
Dataset::load_camera_files(){
	if( path.camera_data.list.empty()){
		for(int i = 0; i < camera_list.cameras.size(); ++i){
				std::string folder = path.root_data_path + "image_" + camera_list.cameras.at(i).id + "/data/";
				path.camera_data.list.push_back(folder);
		}
	}

	for(int i = 0; i < camera_list.cameras.size(); ++i){
		String_list list;

		if(!get_files(path.camera_data.list.at(i), list)) return false;

		camera_file_list.push_back(list);
	}
	return true;
}

bool
Dataset::load_velodyne_to_cam_tf(){
	if(path.tf_velodyne_to_cam0.empty() ) return true; // no path, empty tf, done
	return velodyne_to_cam0.load_file(path.tf_velodyne_to_cam0);
}

bool
Dataset::check()
{
	bool result = true;
	if(camera_list.cameras.size() != path.camera_data.list.size()){
		printf("Check: faild, %lu cameras but %lu camera data",camera_list.cameras.size(), path.camera_data.list.size());
		result = false;
	}

	// Check if all file list contain the same amount of files

	int pointcloud_data_size = pointcloud_file_list.list.size();
	for(int i = 0; i < camera_list.cameras.size(); ++i){
		if(pointcloud_data_size != camera_file_list.at(i).list.size() ){
			printf("Check: faild, directoty: %s, contains %lu, instead of %d files", camera_file_list.at(i).path.c_str(), camera_file_list.at(i).list.size(), pointcloud_data_size );
			result = false;
		}
	}

	if(result){
		std::cout << "Check: ok\n";
	}
	valid = result;
	return valid;
}

bool
Dataset::load_pointcloud_files(){
	if(path.pcl_data.empty()){
		path.pcl_data = path.root_data_path + "velodyne_points/data/";
	}

	if(!get_files(path.pcl_data ,pointcloud_file_list)) return false;
	return true;
}

bool
Dataset::get_files(std::string path, String_list &list){
	list.path = path;
	DIR *dir;
	struct dirent *ent;

	if ((dir = opendir (path.c_str())) != NULL) {
	  /* print all the files and directories within directory */
	  while ((ent = readdir (dir)) != NULL) {
	    if(ent->d_name[0] != '.' ){ // starts with a dot
	    	list.list.push_back(ent->d_name);
		    // printf ("%s\n", ent->d_name);
	    }
	  }
	  closedir (dir);
	  std::sort(list.list.begin(), list.list.end());
	}
	else
	{
	  /* could not open directory */
	  std::cout << "could not open directory: " << path << "\n";
	  return false;
	}

	std::cout << "n: " <<  list.list.size() << " : "<< path << "\n";
	return true;
}

}

} /* namespace image_cloud */
