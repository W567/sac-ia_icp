#ifndef COOK_GEOMETRY_H_
#define COOK_GEOMETRY_H_

#include <vector>
#include <pcl/point_types.h>
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

void findMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<float>& min_max);
void findMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<float>& min_max);
void findZMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& z_min,float& z_max);
void findZMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& z_min,float& z_max);
void getRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& radius);
void getRadius(pcl::PointCloud<pcl::PointXYZ>& cloud,float& radius);
void PCACloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& projection);
void centroidOfCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,Eigen::Vector4f& centroid_of_model);
void centroidOfCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,Eigen::Vector4f& centroid_of_model);


#endif // #ifndef COOK_GEOMETRY_H_
