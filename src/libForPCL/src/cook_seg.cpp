#include "cook_seg.h"

void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane,float threshold)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight(0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (2000);
  seg.setDistanceThreshold (threshold);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;
}

void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max,float threshold)
{
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CIRCLE3D);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (threshold);
  seg.setRadiusLimits (r_min,r_max);
  seg.setInputCloud (cloud_in);
  seg.setInputNormals (normal_in);
  // Obtain the circle inliers and coefficients
  seg.segment (*inliers_circle, *coefficients_circle);
  std::cerr << "Circle coefficients: " << *coefficients_circle << std::endl;
}


int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float tolerance,int max_size,int min_size)
{
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_in);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (tolerance); // 2cm
  ec.setMinClusterSize (min_size);
  ec.setMaxClusterSize (max_size);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_in);
  ec.extract (cluster_indices);

  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_in->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cluster_" << j << ".pcd";
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;
  }
  return j;
}

int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters,float tolerance,int max_size,int min_size)
{
  
}
