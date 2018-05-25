#include "alignment.h"

//input models
void InputStream(std::vector<FeatureCloud>& object_templates,char* fname)
{
  std::ifstream input_stream (fname);
  object_templates.resize (0);
  std::string pcd_filename;
  while (input_stream.good ())
  {
    std::getline (input_stream, pcd_filename);
    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
      continue;

    FeatureCloud template_cloud;
    template_cloud.loadInputCloud (pcd_filename);
    object_templates.push_back (template_cloud);
  }
  input_stream.close ();
}

//first step: align models with targets
void AlignCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,int& best_index,TemplateAlignment::Result& best_alignment)
{
  pcl::console::TicToc tt;
  std::cerr << "Aligning one...\n", tt.tic ();

  FeatureCloud target_cloud;
  target_cloud.setInputCloud (target_in);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < templates_in.size (); ++i)
  {
    template_align.addTemplateCloud (templates_in[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  best_index = template_align.findBestAlignment (best_alignment);

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";

}

//second step: evaluate the alignments and extract the neighbours of the models if it is a good alignment.
int ExtractAlignedModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float r,float voxel_size_in)
{
  pcl::console::TicToc tt;
  std::cerr << "Extracting...\n", tt.tic ();

  pcl::PointCloud<pcl::PointXYZ>::Ptr model(new pcl::PointCloud<pcl::PointXYZ>);
  DownSampleCloud(model_in,model,voxel_size_in);
  float radius=sqrt(3)*voxel_size_in/2;

  std::vector<int> indices;
  indices.clear();
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud (cloud);
  for(size_t i=0;i< model->points.size();++i)
  {
    std::vector<int> indices_part;
    std::vector<float> sqr_distances;
    indices_part.clear();
    kdtree.radiusSearch(model->points[i],radius,indices_part,sqr_distances);
    indices.insert(indices.end(),indices_part.begin(),indices_part.end());
  }

  sort(indices.begin(),indices.end());
  std::vector<int>::iterator new_end=unique(indices.begin(),indices.end());
  indices.erase(new_end,indices.end());

//  pcl::PointCloud<pcl::PointXYZ>::Ptr target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  inliers->indices=indices;
//  ExtractCloud(cloud,inliers,target,false);
  ExtractCloud(cloud,inliers,cloud,true);
//  if(EvaluateTarget(target,model,best_alignment,r)==1)
//  {
    std::cerr << ">> Done: " << tt.toc () << " ms\n";

    return 1;
//  }
//  else
//  {
//    return 0;
//  }
}


int EvaluateTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float radius)
{
  pcl::console::TicToc tt;
  std::cerr << "Evaluating...\n", tt.tic ();

  Eigen::Matrix4f transform_back= best_alignment.final_transformation.inverse();

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud,*transformed_target,transform_back);

  pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
  EstimateNormal(transformed_target,normal);
  pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices);
  SegmentPlane(transformed_target,normal,coefficients_plane,inliers_plane,0.01);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_extracted (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(transformed_target,inliers_plane,cloud_plane_extracted,true);

  // Remove statistical outliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor_plane (new pcl::PointCloud<pcl::PointXYZ>);
  RemoveOutlier(cloud_plane_extracted,cloud_sor_plane);

  // Estimate point normals
//  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals_plane (new pcl::PointCloud<pcl::Normal>);
  if(cloud_sor_plane->points.size()<100)
  {
    std::cout << "points after plane extracting are not enough for circle segmentation" << std::endl;
    std::cerr << ">> Done: " << tt.toc () << " ms\n";
    return 0;
  }
  EstimateNormal(cloud_sor_plane,normal);

  // Create the segmentation object for circle segmentation and set all the parameters
  pcl::ModelCoefficients::Ptr coefficients_circle (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_circle (new pcl::PointIndices);
  SegmentCircle3D(cloud_sor_plane,normal,coefficients_circle,inliers_circle,0.8*radius,1.2*radius,0.01);

  float cir_x = coefficients_circle->values[0];
  float cir_y = coefficients_circle->values[1];
  float cir_na = coefficients_circle->values[4];
  float cir_nb = coefficients_circle->values[5];
  float cir_nc = coefficients_circle->values[6];

  //ShowCloud(cloud_sor_plane);
  // Extract the circle out of cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_circle (new pcl::PointCloud<pcl::PointXYZ>);
  ExtractCloud(cloud_sor_plane,inliers_circle,cloud_circle);
  //ShowCloud(cloud_circle);

  std::cout << "size of circle: " << cloud_circle->points.size() << std::endl ;
  std::cout << "size of cloud with plane extracted : " << cloud_sor_plane->points.size() << std::endl;

  if( fabs(cir_na) < 0.1 && fabs(cir_nb) < 0.1 && fabs(cir_nc) > 0.9)
  {
    std::cout << "error is : " << cir_x*cir_x + cir_y*cir_y << std::endl;
    if( (cir_x*cir_x + cir_y*cir_y) <0.0001 )
    {
      if( cloud_circle->points.size() > 0.5 * cloud_sor_plane->points.size() )
      {
        std::cout << "valid target" << std::endl;
        std::cerr << ">> Done: " << tt.toc () << " ms\n";
        return 1; //valid target
      }
      else
      {
        std::cout << "quantity unsatisfied" << std::endl;
        std::cerr << ">> Done: " << tt.toc () << " ms\n";
        return 0;  //quantity unsatisfied
      }
    }
    else
    {
      std::cout << "centroid unsatisfied" << std::endl;
      std::cerr << ">> Done: " << tt.toc () << " ms\n";
      return 0;
    }
  }
  else
  {
    std::cout << "normal unsatisfied" << std::endl;
    std::cerr << ">> Done: " << tt.toc () << " ms\n";
    return 0;  // normal of the circle unsatisfied
  }
}


//entire procedure:
//
int AlignAllModels(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,float fit_threshold,std::vector<PlateInformation>& infor)
{
  pcl::console::TicToc tt;
  std::cerr << "Aligning ALL...\n", tt.tic ();

  int j = 0;
  int i = 0;
  float fit_score = 0;
  int best_index=-1;
  TemplateAlignment::Result best_alignment;
  pcl::PCDWriter writer;
  infor.clear();

  while(fit_score <= fit_threshold)
  {
    // Assign to the target FeatureCloud
    std::cout<<"--------------------------------aligncloud-----------------------------------"<<std::endl;
    AlignCloud(target_in,templates_in,best_index,best_alignment);
    fit_score = best_alignment.fitness_score;
    if(fit_score <= fit_threshold)
    {
      const FeatureCloud &best_template = templates_in[best_index];

      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);

      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      ICPCloud(transformed_cloud,target_in,icp_cloud,icp);

      //WriteCloud(icp_cloud,"icpCloud.pcd");
      std::stringstream ss1;
      ss1 << "icp_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss1.str (), *icp_cloud, false);

      //pcl::io::savePCDFileBinary ("output.pcd", *transformed_cloud);
      float radius;
      getRadius(*best_template.getPointCloud (),radius);
      if(ExtractAlignedModel(target_in,icp_cloud,best_alignment,radius,0.01f)==1)
      {
        Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
        PlateInformation oneInfor;
        oneInfor.x=translation[0];
        oneInfor.y=translation[1];
        oneInfor.z=translation[2];

        getRadius(*best_template.getPointCloud (),oneInfor.radius);
        infor.push_back(oneInfor);

        i=0;
        std::stringstream ss;
        ss << "model_aligned_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *transformed_cloud, false); //*
        printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
        std::cout<<"find a plate "<<std::endl;
        j++;
      }
      else
      {
        i++;
        std::cout<<"find a plane"<<std::endl;
      }

    }
    if(i>0)           //----------------------------------------------------------------找到平面后的匹配次数
    {
      std::cout<<"find plane 1 time and out" << std::endl;
      std::cerr << ">> Done: " << tt.toc () << " ms\n";
      return j;
    }
  }
  std::cout<<"didn't find any target and out"<<std::endl;
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  return j;
}


void
ICPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_in,pcl::PointCloud<pcl::PointXYZ>::Ptr model_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& target_out,pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp)
{
  pcl::console::TicToc tt;
  std::cerr << "Aligning ALL...\n", tt.tic ();
  icp.setInputSource(target_in);
  icp.setInputTarget(model_in);
  icp.setMaximumIterations(50);
  icp.setRANSACIterations (50);
  //icp.setMaxCorrespondenceDistance(0.01f*0.1f);
  icp.align(*target_out);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
}
