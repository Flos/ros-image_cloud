#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/range_image/range_image.h>

#include <pcl/features/range_image_border_extractor.h>

#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_cloud/common/small_helpers.hpp>

#ifndef PCL_RANGE_BORDERS_FILTER_H_
#define PCL_RANGE_BORDERS_FILTER_H_

//#define visual_debug

namespace filter_3d{

template <typename PointT>
inline void
range_borders(
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		double angular_resolution // Angular resolution for image
)
{

	//float angular_resolution = 0.5f;
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	bool setUnseenToMaxRange = false;
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;

	boost::shared_ptr<pcl::PointCloud<PointT> > in_ptr(in.makeShared());

	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------


	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;
	angular_resolution = pcl::deg2rad (angular_resolution);

	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
	float nois_level = 0.1;

	range_image.createFromPointCloud(*in_ptr, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
								   scene_sensor_pose, coordinate_frame, nois_level, min_range, border_size);
	range_image.integrateFarRanges(far_ranges);

	if (setUnseenToMaxRange) range_image.setUnseenToMaxRange ();

	// -------------------------
	// -----Extract borders-----
	// -------------------------
	pcl::RangeImageBorderExtractor border_extractor (&range_image);
	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
	border_extractor.compute(border_descriptions);

#ifdef visual_debug

	  pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
	                                            veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
	                                            shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
	  pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
	                                      & veil_points = * veil_points_ptr,
	                                      & shadow_points = *shadow_points_ptr;
#endif

	for (int y=0; y< (int)range_image.height; ++y)
	{
		for (int x=0; x< (int)range_image.width; ++x)
		{
			if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
			{
				PointT point;
				point.x = range_image.points[y*range_image.width + x].x;
				point.y = range_image.points[y*range_image.width + x].y;
				point.z = range_image.points[y*range_image.width + x].z;
				point.intensity = range_image.points[y*range_image.width + x].range;
				out.points.push_back(point);
			}
#ifdef visual_debug
			  if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
			        border_points.points.push_back (range_image.points[y*range_image.width + x]);
			      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
			        veil_points.points.push_back (range_image.points[y*range_image.width + x]);
			      if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
			        shadow_points.points.push_back (range_image.points[y*range_image.width + x]);
#endif
		}
	}

#ifdef visual_debug
	 pcl::visualization::RangeImageVisualizer* range_image_borders_widget = NULL;
	  range_image_borders_widget =
	    pcl::visualization::RangeImageVisualizer::getRangeImageBordersWidget (range_image, -std::numeric_limits<float>::infinity (), std::numeric_limits<float>::infinity (), false,
	                                                                          border_descriptions, "Range image with borders");
	  // -------------------------------------
	  // --------------------------------------------
	    // -----Open 3D viewer and add point cloud-----
	    // --------------------------------------------
	    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	    viewer.setBackgroundColor (1, 1, 1);
//	    viewer.addCoordinateSystem (1.0f, "global");
	    pcl::visualization::PointCloudColorHandlerCustom<PointT> point_cloud_color_handler (in_ptr, 0, 0, 0);
	    viewer.addPointCloud (in_ptr, point_cloud_color_handler, "original point cloud");
	  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler (border_points_ptr, 0, 255, 0);
	    viewer.addPointCloud<pcl::PointWithRange> (border_points_ptr, border_points_color_handler, "border points");
	    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler (veil_points_ptr, 255, 0, 0);
	    viewer.addPointCloud<pcl::PointWithRange> (veil_points_ptr, veil_points_color_handler, "veil points");
	    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
	    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler (shadow_points_ptr, 0, 255, 255);
	    viewer.addPointCloud<pcl::PointWithRange> (shadow_points_ptr, shadow_points_color_handler, "shadow points");
	    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");

	  //--------------------
	  // -----Main loop-----
	  //--------------------

	viewer.spinOnce ();
	pcl_sleep(0.01);
#endif

	// Save filtered output
	std::cout << "In: " << in.size() << " Filtered: " << out.size () << " data points." << std::endl;
}

}


#endif
