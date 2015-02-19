#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#ifndef SEGMENTATION_H_
#define SEGMENTATION_H_

namespace filter_3d{

template <typename PointT>
void segmentation(const pcl::PointCloud<PointT> &in, pcl::PointCloud<PointT> &out){

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.05);
	seg.setEpsAngle( 0.05);

	seg.setInputCloud(in.makeShared());
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size () == 0)
	{
	  std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
	}

	// Extract the planar inliers from the input cloud
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(in.makeShared());
	extract.setIndices (inliers);
	extract.setNegative (false);

	// Get the points associated with the planar surface
	extract.filter (out);
	//std::cout << "PointCloud representing the planar component: " << out.points.size () << " data points." << std::endl;

	// Remove the planar inliers, extract the rest
//	extract.setNegative (true);
//	extract.filter (*cloud_f);
//	*cloud_filtered = *cloud_f;
}

template <typename PointT>
void euclidian_cluster(pcl::PointCloud<PointT> &in, pcl::PointCloud<PointT> &out){
//	// Create the filtering object: downsample the dataset using a leaf size of 1cm
//	  pcl::VoxelGrid<PointT> vg;
//	  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
//	  vg.setInputCloud (in.makeShared());
//	  vg.setLeafSize (0.01f, 0.01f, 0.01f);
//	  vg.filter (*cloud_filtered);
//	  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
//
//	  // Create the segmentation object for the planar model and set all the parameters
//	  pcl::SACSegmentation<PointT> seg;
//	  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//	  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//	  pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
//
//	  seg.setOptimizeCoefficients (true);
//	  seg.setModelType (pcl::SACMODEL_PLANE);
//	  seg.setMethodType (pcl::SAC_RANSAC);
//	  seg.setMaxIterations (100);
//	  seg.setDistanceThreshold (0.05);
//
//	  int i=0, nr_points = (int) cloud_filtered->points.size ();
//	  while (cloud_filtered->points.size () > 0.3 * nr_points)
//	  {
//	    // Segment the largest planar component from the remaining cloud
//	    seg.setInputCloud (cloud_filtered);
//	    seg.segment (*inliers, *coefficients);
//	    if (inliers->indices.size () == 0)
//	    {
//	      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
//	      break;
//	    }
//
//	    // Extract the planar inliers from the input cloud
//	    pcl::ExtractIndices<PointT> extract;
//	    extract.setInputCloud (cloud_filtered);
//	    extract.setIndices (inliers);
//	    extract.setNegative (false);
//
//	    // Get the points associated with the planar surface
//	    extract.filter (*cloud_plane);
//	    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
//
//	    // Remove the planar inliers, extract the rest
//	    extract.setNegative (true);
//	    extract.filter (*cloud_f);
//	    *cloud_filtered = *cloud_f;
//	  }
//
//	  // Creating the KdTree object for the search method of the extraction
//	  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//	  tree->setInputCloud (cloud_filtered);
//
//	  std::vector<pcl::PointIndices> cluster_indices;
//	  pcl::EuclideanClusterExtraction<PointT> ec;
//	  ec.setClusterTolerance (0.02); // 2cm
//	  ec.setMinClusterSize (100);
//	  ec.setMaxClusterSize (25000);
//	  ec.setSearchMethod (tree);
//	  ec.setInputCloud (cloud_filtered);
//	  ec.extract (cluster_indices);
//
//	  int j = 0;
//	  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
//	  {
//	    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
//	    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
//	      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//	    cloud_cluster->width = cloud_cluster->points.size ();
//	    cloud_cluster->height = 1;
//	    cloud_cluster->is_dense = true;
//
//	    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//	    std::stringstream ss;
//	    ss << "cloud_cluster_" << j << ".pcd";
//	    writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
//	    j++;
//	  }

}

}
#endif
