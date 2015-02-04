#include <pcl/common/common.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/don.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.h>
#include <eigen3/Eigen/Core>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_cloud/common/small_helpers.hpp>

#ifndef PCL_NORMAL_DIFF_FILTER_H_
#define PCL_NORMAL_DIFF_FILTER_H_

namespace filter_3d{

template <typename PointT>
inline void
normal_diff_filter(
		const pcl::PointCloud<PointT> &in,
		pcl::PointCloud<PointT> &out,
		double scale1, ///The smallest scale to use in the DoN filter.
		double scale2, ///The largest scale to use in the DoN filter.
		double threshold ///The minimum DoN magnitude to threshold by
)
{

	boost::shared_ptr<pcl::PointCloud<PointT> > in_ptr(in.makeShared());


	boost::shared_ptr<pcl::search::KdTree<PointT> > tree_n;
	tree_n.reset( new pcl::search::KdTree<PointT>(false) );
	tree_n->setInputCloud(in_ptr);
	tree_n->setEpsilon(0.5);

	if (scale1 >= scale2)
	{
	    printf("Error: Large scale must be > small scale!");
	    return;
	}

	// Compute normals using both small and large scales at each point
	pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
	ne.setInputCloud (in_ptr);
	ne.setSearchMethod (tree_n);

	/**
	* NOTE: setting viewpoint is very important, so that we can ensure
	* normals are all pointed in the same direction!
	*/
	ne.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());

	// calculate normals with the small scale
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale (new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch (scale1);
	ne.compute (*normals_small_scale);

	// calculate normals with the large scale
	pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointNormal>);

	ne.setRadiusSearch (scale2);
	ne.compute (*normals_large_scale);

	// Create output cloud for DoN results
	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointNormal>);
	pcl::copyPointCloud<PointT, pcl::PointNormal>(in, *doncloud);

	// Create DoN operator
	pcl::DifferenceOfNormalsEstimation<PointT, pcl::PointNormal, pcl::PointNormal> don;
	don.setInputCloud(in_ptr);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);

	if (!don.initCompute ())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		return;
	}

	// Compute DoN
	don.computeFeature (*doncloud);


	// Build the condition for filtering
	pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond (
			new pcl::ConditionOr<pcl::PointNormal> ()
	);
	range_cond->addComparison (pcl::FieldComparison<pcl::PointNormal>::ConstPtr (
							   new pcl::FieldComparison<pcl::PointNormal> ("curvature", pcl::ComparisonOps::GT, threshold))
							 );
	// Build the filter

	pcl::ConditionalRemoval<pcl::PointNormal> condrem (range_cond, true);
	condrem.setInputCloud(doncloud);


	pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered (new pcl::PointCloud<pcl::PointNormal>);

	// Apply filter
	condrem.filter(*doncloud_filtered);

	boost::shared_ptr<pcl::PointIndices> indices(new pcl::PointIndices);
	condrem.getRemovedIndices(*indices);

	std::cout << "Indices: " << indices->indices.size() << " Filtered: " << doncloud_filtered->size () << " data points." << std::endl;

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(in_ptr);
	extract.setIndices(indices);
	extract.setNegative (true);
	extract.filter(out);

	// Save filtered output
	std::cout << "In: " << in.size() << " Filtered: " << out.size () << " data points." << std::endl;
}

}


#endif
