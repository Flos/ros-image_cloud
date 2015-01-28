#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <common/small_helpers.hpp>
#include <opencv2/core/core.hpp>
#include <common/type.hpp>

#ifndef PROJECT_2D_H_
#define PROJECT_2D_H_

namespace project2d
{

inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<pcl::PointXYZI> &in,
		pcl::PointCloud<pcl::PointXY> &out_2d,
		pcl::PointCloud<pcl::PointXYZI> &out_3d,
		unsigned int camera_width = 1024,
		unsigned int camera_height = 768
		)
{
	BOOST_FOREACH (const pcl::PointXYZI& pt, in.points)
	{
		if( pt.z > 1) { // min distance from camera 1m

			cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( between<int>(0, point_image.x, camera_width )
				&& between<int>( 0, point_image.y, camera_height )
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



enum Field{
	INTENSITY = 0,
	DEPTH = 1
};


template <typename PointT>
void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		cv::Mat &image,
		Field field = DEPTH,
		int point_size = 1,
		float min_distance = 1
		)
{
	BOOST_FOREACH (const PointT& pt, in.points)
	{
		if( pt.z > min_distance) { // min distance from camera 1m

			cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

			if( between<int>(0, point_image.x, image.cols )
				&& between<int>( 0, point_image.y, image.rows )
			)
			{
				switch(field){
					default:
					case DEPTH:
							cv::circle(image, point_image, point_size, cv::Scalar(pt.z), -1);
						break;
					case INTENSITY:
							cv::circle(image, point_image, point_size, cv::Scalar(pt.intensity), -1);
						break;
				}
			}
		}
	}
}

template <typename PointT>
void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<PointT> &in,
		std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > &out,
		int image_width,
		int image_height)
{
		BOOST_FOREACH (pcl::PointXYZI& pt, in.points){
			if( pt.z > 1) { // min distance from camera 1m

				cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

				if( between<int>(0, point_image.x, image_width )
					&& between<int>( 0, point_image.y, image_height )
				)
				{
					// Point in image push to 2d and 3d point
					out[point_image.x][point_image.y].reset(new pcl::PointXYZI(pt));
				}
			}
		}
}

template <typename PointT>
void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<PointT> &in,
		std::vector<std::vector<boost::shared_ptr<pcl::PointXYZI> > > &out_vector,
		Projected_Pointclouds<PointT> &out,
		int image_width,
		int image_height)
{
		out.image_size.width = image_width;
		out.image_size.height = image_height;

		BOOST_FOREACH (pcl::PointXYZI& pt, in.points){
			if( pt.z > 1) { // min distance from camera 1m

				cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

				if( between<int>(0, point_image.x, image_width )
					&& between<int>( 0, point_image.y, image_height )
				)
				{
					// Point in image push to 2d and 3d point
					out_vector[point_image.x][point_image.y].reset(new pcl::PointXYZI(pt));
					out.points.push_back(Projected_Point<PointT>(pt, point_image));
				}
			}
		}
}

template <typename PointT>
inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		pcl::PointCloud<PointT> &in,
		Projected_Pointclouds<PointT> &out,
		int image_width,
		int image_height)
{
		out.image_size.width = image_width;
		out.image_size.height = image_height;

		BOOST_FOREACH (pcl::PointXYZI& pt, in.points){
			if( pt.z > 1) { // min distance from camera 1m

				cv::Point2f point_image = camera_model.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));

				if( between<int>(0, point_image.x, image_width )
					&& between<int>( 0, point_image.y, image_height )
				)
				{
					out.points.push_back(Projected_Point<PointT>(pt, point_image));
				}
			}
		}
}


template <typename PointT>
void
project_2d(
		std::vector<std::vector<boost::shared_ptr<PointT> > > &in,
		cv::Mat &out,
		Field field = INTENSITY,
		int point_size = 1
		)
{
		for(int y = 0; y < in[0].size(); y++)
		{
			for(int x = 0; x < in.size(); x++)
			{
				cv::Point3i value;
				if( in[x][y]){ /* found something */
					switch(field){
						default:
						case DEPTH:
								cv::circle(out, cv::Point2i(x,y), point_size, cv::Scalar(in[x][y]->z), -1);
							break;
						case INTENSITY:
								cv::circle(out, cv::Point2i(x,y), point_size, cv::Scalar(in[x][y]->intensity), -1);
							break;
					}
				}
			}
		}
}

}
#endif
