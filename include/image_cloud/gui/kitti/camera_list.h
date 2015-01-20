/*
 * camera_list.h
 *
 *  Created on: 20.01.2015
 *      Author: fnolden
 */

#include <gui/kitti/serializable.h>
#include <gui/kitti/camera.h>

#ifndef SRC_GUI_KITTI_CAMERA_LIST_H_
#define SRC_GUI_KITTI_CAMERA_LIST_H_

namespace image_cloud {

namespace kitti{

class Camera_list: public Serializable {
public:
	Camera_list();
	virtual ~Camera_list();

	float corner_dist;
	std::vector<Camera> cameras;

	std::string to_string();
	bool load( std::istream &stream);
};

}

} /* namespace image_cloud */

#endif /* SRC_GUI_KITTI_CAMERA_LIST_H_ */
