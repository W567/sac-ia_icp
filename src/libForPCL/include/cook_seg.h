#ifndef COOK_SEG_H_
#define COOK_SEG_H_

#include <pcl/point_types.h>
#include <string>
#include <iostream>
#include <vector>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

void SegmentPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_plane,pcl::PointIndices::Ptr& inliers_plane,float threshold);
void SegmentCircle3D(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::Normal>::Ptr& normal_in,pcl::ModelCoefficients::Ptr& coefficients_circle,pcl::PointIndices::Ptr& inliers_circle,float r_min,float r_max,float threshold);
int ExtractClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,float tolerance,int max_size = 25000,int min_size = 100);

#endif //ifndef COOK_SEG_H_
