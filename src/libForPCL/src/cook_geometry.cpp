#include "cook_geometry.h"

void centroidOfCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,Eigen::Vector4f& centroid_of_model)
{
  centroidOfCloud(*cloud,centroid_of_model);
}

void centroidOfCloud(pcl::PointCloud<pcl::PointXYZ>& cloud,Eigen::Vector4f& centroid_of_model)
{
  pcl::compute3DCentroid<pcl::PointXYZ> (cloud,centroid_of_model);
  std::cout << "centroid of the model is: " << std::endl;
  std::cout << centroid_of_model << std::endl;
}

void PCACloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& projection)
{
  pcl::PCA<pcl::PointXYZ> pca (new pcl::PCA<pcl::PointXYZ>);
  pca.setInputCloud(cloud);
  pca.project(*cloud,*projection);
}

void findMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,std::vector<float>& min_max)
{
  findMinMax(*cloud,min_max);
}

void findMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,std::vector<float>& min_max)
{
  min_max.clear();
  float x_min=FLT_MAX,y_min=FLT_MAX,z_min=FLT_MAX;
  float x_max=-FLT_MAX,y_max=-FLT_MAX,z_max=-FLT_MAX;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      y_min = y_min < cloud.points[i].y ? y_min : cloud.points[i].y;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;

      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
      y_max = y_max > cloud.points[i].y ? y_max : cloud.points[i].y;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      x_min = x_min < cloud.points[i].x ? x_min : cloud.points[i].x;
      y_min = y_min < cloud.points[i].y ? y_min : cloud.points[i].y;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;

      x_max = x_max > cloud.points[i].x ? x_max : cloud.points[i].x;
      y_max = y_max > cloud.points[i].y ? y_max : cloud.points[i].y;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  min_max.push_back(x_min);
  min_max.push_back(x_max);
  min_max.push_back(y_min);
  min_max.push_back(y_max);
  min_max.push_back(z_min);
  min_max.push_back(z_max);
}

void findZMinMax(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& z_min,float& z_max)
{
  findZMinMax(*cloud,z_min,z_max);
}

void findZMinMax(const pcl::PointCloud<pcl::PointXYZ>& cloud,float& z_min,float& z_max)
{
  z_min=FLT_MAX;
  z_max=-FLT_MAX;
  // If the data is dense, we don't need to check for NaN
  if (cloud.is_dense)
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
  // NaN or Inf values could exist => check for them
  else
  {
    for (size_t i = 0; i < cloud.points.size (); ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud.points[i].x) || !pcl_isfinite (cloud.points[i].y) || !pcl_isfinite (cloud.points[i].z))
        continue;
      z_min = z_min < cloud.points[i].z ? z_min : cloud.points[i].z;
      z_max = z_max > cloud.points[i].z ? z_max : cloud.points[i].z;
    }
  }
}

void getRadius(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float& radius)
{
  getRadius(*cloud,radius);
}

void getRadius(pcl::PointCloud<pcl::PointXYZ>& cloud,float& radius)
{
  radius=0;
  std::vector<float> min_max;
  findMinMax(cloud,min_max);
//  std::cout << "min_max" << min_max[0] <<";"<< min_max[1] << ";" << min_max[2] << ";" << min_max[3] << std::endl;
  float average=0;
  average=(min_max[1]-min_max[0]+min_max[3]-min_max[2])/4;
  std::cout<<"average : " <<average << std::endl;
  int i=0;
  if(-min_max[0]<average)
  {
    radius-=min_max[0];
    i++;
  }
  if(min_max[1]<average)
  {
    radius+=min_max[1];
    i++;
  }
  if(-min_max[2]<average)
  {
    radius-=min_max[2];
    i++;
  }
  if(min_max[3]<average)
  {
    radius+=min_max[3];
    i++;
  }
  radius=radius/i;
  std::cout << "radius:  "  << radius << "  i:  "<< i << std::endl;
}
