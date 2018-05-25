#ifndef COOK_BASIS_H_
#define COOK_BASIS_H_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

void PrePassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float min,float max,std::string axis = "z");
void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float size = 0.003f);
void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int meanK = 50, float thresh = 0.1);
void EstimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_out, int rangeK = 50);
void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float x,float y,float z,std::string axis,float angle);
void ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointIndices::Ptr inliers_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,bool state = false);

#endif //ifndef COOK_IO_H_
