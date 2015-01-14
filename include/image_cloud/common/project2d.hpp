#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>

#ifndef PROJECT_2D_H_
#define PROJECT_2D_H_

inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
		pcl::PointCloud<pcl::PointXY> &out_2d,
		pcl::PointCloud<pcl::PointXYZI> &out_3d,
		unsigned int camera_width = 1024,
		unsigned int camaer_height = 768
		)
{
	BOOST_FOREACH (const pcl::PointXYZI& pt, in->points)
	{
		if( pt.z > 1) { // min distance from camera 1m

			cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( ( point_image.x > 0
					&& point_image.x < camera_width )
					&& ( point_image.y > 0
					&& point_image.y < camaer_height )
			)
			{
				// Point in image push to 2d and 3d point
				pcl::PointXY pt_2d;
				pt_2d.x = point_image.x;
				pt_2d.y = point_image.y;
				out_2d.push_back(pt_2d);
				out_3d.push_back(pt);
			}
		}

	}

}

#endif
