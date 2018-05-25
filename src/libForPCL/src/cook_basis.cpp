#include "cook_basis.h"

void PrePassThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float min,float max,std::string axis)
{
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName (axis); //'x' 'y' 'z'
  pass.setFilterLimits (min,max);
  pass.filter(*cloud_out);
  std::cerr << "PointCloud after filtering(passthrough) has: " << cloud_out->points.size() << " data points." << std::endl;
}

void DownSampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float size)
{
  pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
  voxelGrid.setInputCloud(cloud_in);
  voxelGrid.setLeafSize(size,size,size); //0.003f
  voxelGrid.filter(*cloud_out);
  std::cerr << "PointCloud after downsampling has: " << cloud_out->points.size() << " data points." << std::endl;
}

void RemoveOutlier(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out, int meanK, float thresh)
{
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud_in);
  sor.setMeanK(meanK);  //50
  sor.setStddevMulThresh(thresh); //0.1
  sor.filter(*cloud_out);
  std::cerr << "cloud after StatisticalOutlierRemoval has: " << cloud_out->points.size() << std::endl;
}

void EstimateNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in, pcl::PointCloud<pcl::Normal>::Ptr normal_out, int rangeK)
{
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
  ne.setInputCloud (cloud_in);
  ne.setSearchMethod (tree);
  ne.setKSearch (rangeK);
  ne.compute (*normal_out);
}

void TransformCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,float x,float y,float z,std::string axis,float angle)
{
  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform.translation() << x,y,z; //0.033, -0.075, -0.45;

  //The same rotation matrix as before; theta radians around Z axis
  if(axis == "x")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitX()));}
  else if(axis == "y")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitY()));}
  else if(axis == "z")
  {transform.rotate (Eigen::AngleAxisf (angle,Eigen::Vector3f::UnitZ()));}
  else {std::cout << "input an illegal parameter" << std::endl;}

  // Print the transformation
  printf ("\nusing an Affine3f\n");
  std::cout << transform.matrix() << std::endl;

  pcl::transformPointCloud (*cloud_in, *cloud_out, transform);
}

void ExtractCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,pcl::PointIndices::Ptr inliers_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,bool state)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud_in);
  extract.setIndices (inliers_in);
  extract.setNegative (state);   //true: rest   false: inliers
  extract.filter (*cloud_out);
}
