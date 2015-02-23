#include <image_cloud/common/small_helpers.hpp>
#include <image_cloud/common/type.hpp>

#include <pcl/common/common.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/core/core.hpp>

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
	for(int i = 0; i < in.points.size(); ++i)
	{
		if( in.points.at(i).z > 1) { // min distance from camera 1m

			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

			if( between<int>(0, point_image.x, camera_width )
				&& between<int>( 0, point_image.y, camera_height )
			)
			{
				// Point in image push to 2d and 3d point
				pcl::PointXY pt_2d;
				pt_2d.x = point_image.x;
				pt_2d.y = point_image.y;
				out_2d.push_back(pt_2d);
				out_3d.push_back(in.points.at(i));
			}
		}
	}
}



enum Field{
	INTENSITY = 0,
	DEPTH = 1
};


template <typename PointT>
inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		cv::Mat &image,
		Field field = DEPTH,
		int point_size = 1,
		float min_distance = 1
		)
{
	for(int i = 0; i < in.points.size(); ++i)
	{
		if( in.points.at(i).z > min_distance) { // min distance from camera 1m

			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

			if( between<int>(0, point_image.x, image.cols )
				&& between<int>( 0, point_image.y, image.rows )
			)
			{
				switch(field){
					default:
					case DEPTH:
							cv::circle(image, point_image, point_size, cv::Scalar(255 - in.points.at(i).z), -1);
						break;
					case INTENSITY:
							cv::circle(image, point_image, point_size, cv::Scalar(255 - in.points.at(i).intensity), -1);
						break;
				}
			}
		}
	}
}

template <typename PointT>
inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &out,
		bool shortest_distance = true
		)
{
	for(int i = 0; i < in.points.size(); ++i)
	{
		if( in.points.at(i).z > 1) { // min distance from camera 1m

			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

			if( between<int>(0, point_image.x, out.size() )
				&& between<int>( 0, point_image.y, out[0].size() )
			)
			{
				// prefer lower distance points (they are more likely to be a edge)
				if(shortest_distance && out[point_image.x][point_image.y] && out[point_image.x][point_image.y]->z < in.points.at(i).z){
					continue;
				}
				// Point in image push to 2d and 3d point
				out[point_image.x][point_image.y].reset(new PointT(in.points.at(i)));
			}
		}
	}
}

template <typename PointT>
inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		std::vector<std::vector<boost::shared_ptr<PointT> > > &out_vector,
		Projected_pointcloud<PointT> &out,
		int image_width,
		int image_height)
{
	assert(out_vector.size() == image_width);
	assert(out_vector[0].size() == image_height);
	out.image_size.width = image_width;
	out.image_size.height = image_height;

	for(int i = 0; i < in.points.size(); ++i)
	{
		if( in.points.at(i).z > 1) { // min distance from camera 1m

			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

			if( between<int>(0, point_image.x, image_width )
				&& between<int>( 0, point_image.y, image_height )
			)
			{
				// Point in image push to 2d and 3d point
				out_vector[point_image.x][point_image.y].reset(new PointT(in.points.at(i)));
				out.points.push_back(Projected_Point<PointT>(in.points.at(i), point_image));
			}
		}
	}
}

template <typename PointT>
inline void
project_2d(
		const image_geometry::PinholeCameraModel &camera_model,
		const pcl::PointCloud<PointT> &in,
		Projected_pointcloud<PointT> &out,
		int image_width,
		int image_height)
{
	out.image_size.width = image_width;
	out.image_size.height = image_height;

	for(int i = 0; i < in.points.size(); ++i)
	{
		if( in.points.at(i).z > 1) { // min distance from camera 1m

			cv::Point2i point_image = camera_model.project3dToPixel(cv::Point3d(in.points.at(i).x, in.points.at(i).y, in.points.at(i).z));

			if( between<int>(0, point_image.x, image_width )
				&& between<int>( 0, point_image.y, image_height )
			)
			{
				out.points.push_back(Projected_Point<PointT>(in.points.at(i), point_image));
			}
		}
	}
}


template <typename PointT>
inline void
project_2d(
		const std::vector<std::vector<boost::shared_ptr<PointT> > > &in,
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

template<typename PointT>
	struct Points2d{
		std::vector<std::vector<long unsigned int> > indices;
		boost::shared_ptr<pcl::PointCloud<PointT> > points_ptr;

		int size_x(){
			return indices.size();
		}

		int size_y(){
			return indices.at(0).size();
		}

		Points2d(){
		}

		Points2d(int size_x, int size_y){
			init(size_x, size_y);
		}

		void init(int size_x, int size_y){
			indices = std::vector<std::vector<long unsigned int> > ( size_x, std::vector< long unsigned int >(size_y));

			assert(size_x == this->size_x());
			assert(size_y == this->size_y());
		}

		void init(const image_geometry::PinholeCameraModel& camera_model,
				const pcl::PointCloud<PointT> &in_points,
				int rows,
				int cols)
		{
			init(cols, rows);
			project2d(camera_model, in_points, cols, rows);
		}

		void init(const image_geometry::PinholeCameraModel& camera_model,
						const pcl::PointCloud<PointT> &in_points,
						cv::Mat & image,
						project2d::Field field,
						int point_size = 1)
		{
			init(image.cols, image.rows);
			points_ptr = in_points.makeShared();
			project2d(camera_model, in_points, image, field, point_size);
		}

		const PointT& at(int x, int y){
			return points_ptr->points.at(indices[x][y]);
		}

		const PointT& at(const long unsigned int &indice){
			return points_ptr->points.at(indice);
		}

		void insert(const cv::Point2i cv_point, const long unsigned int &indice){
			insert(cv_point.x, cv_point.y, indice);
		}

		void insert(const int x, const int y, const long unsigned int &indice){
			if(at(indice).z < at(x,y).z || indices.at(x).at(y) == 0)
			{
				indices[x][y] = indice;
			}
		}

		void project2d(const image_geometry::PinholeCameraModel& camera_model,
				const pcl::PointCloud<PointT> &in,
				int rows,
				int cols)
		{
			points_ptr = in.makeShared();
			init(cols, rows);

			for (long unsigned int i = 0; i < in.points.size(); ++i) {
				const PointT* pt = &in.points.at(i);
				if (pt->z > 1) {
					// min distance from camera 1m
					cv::Point2i point_image = camera_model.project3dToPixel(
							cv::Point3d(pt->x, pt->y, pt->z));
					if (between<int>(0, point_image.x, cols)
							&& between<int>(0, point_image.y, rows)) {
						// Sort all points into image
						{
							insert(point_image, i);
						}
					}
				}
			}
		}

		void project2d(const image_geometry::PinholeCameraModel& camera_model,
						const pcl::PointCloud<PointT> &in,
						cv::Mat &image,
						project2d::Field field = DEPTH,
						int point_size = 1,
						float min_z = 1,
						float max_z = 0)
		{
			points_ptr = in.makeShared();
			init(image.cols, image.rows);

			for (long unsigned int i = 0; i < in.points.size(); ++i) {

				const PointT* pt = &in.points.at(i);

				if (pt->z < min_z) continue;
				if ( max_z != 0 && pt->z > max_z) continue;

				// min distance from camera 1m
				cv::Point2i point_image = camera_model.project3dToPixel(
						cv::Point3d(pt->x, pt->y, pt->z));

				if (between<int>(0, point_image.x, image.cols)
						&& between<int>(0, point_image.y, image.rows)) {

					// Insert point
					insert(point_image, i);
					//std::cout << i << "\t" << at(point_image.x,point_image.y) << "\n";

					// Draw into image
					switch(field){
						default:
						case DEPTH:
								cv::circle(image, point_image, point_size, cv::Scalar(255 - in.points.at(i).z), -1);
							break;
						case INTENSITY:
								cv::circle(image, point_image, point_size, cv::Scalar(255 - in.points.at(i).intensity), -1);
							break;
					}
				}
			}
		}

		template<typename imageType>
		void get_points(const cv::Mat &image, pcl::PointCloud<PointT> &out, float threshold = 1)
		{
			for (int r = 0; r < image.rows; ++r){
				for(int c = 0; c < image.cols; ++c){

					if(image.at<imageType>(r,c) > threshold
							&& indices[c][r] != 0
							){
						//std::cout << at(c,r) << "\n";
						out.push_back( at(c,r) ); // take the point with the shortest distance
					}
				}
			}
		}
	};

}
#endif
