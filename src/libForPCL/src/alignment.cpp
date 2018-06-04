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
  printf ("Best fitness score: %f\n", best_alignment.icp_score);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
}

//second step: evaluate the alignments and extract the neighbours of the models if it is a good alignment.
int ExtractAlignedModel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr& model_in,TemplateAlignment::Result& best_alignment,float voxel_size_in)
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

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
  inliers->indices=indices;
  ExtractCloud(cloud,inliers,cloud,true);
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  return 1;
}

//entire procedure:
//
int AlignAllModels(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_in,std::vector<FeatureCloud>& templates_in,float fit_threshold,std::vector<PlateInformation>& infor)
{
  pcl::console::TicToc tt;
  std::cerr << "Aligning ALL...\n", tt.tic ();
  int k = 0;
  int j = 0;
  int try_times = 0;
  float fit_score = 0.0f;
//  float icp_score = 0.0f;
  int best_index=-1;
  TemplateAlignment::Result best_alignment;
  pcl::PCDWriter writer;
  infor.clear();

  while(fit_score <= fit_threshold || try_times < 1)
  {
    // Assign to the target FeatureCloud
    std::cout<<"--------------------------------aligncloud-----------------------------------"<<std::endl;
    AlignCloud(target_in,templates_in,best_index,best_alignment);
    fit_score = best_alignment.icp_score;

    const FeatureCloud &best_template = templates_in[best_index];
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*best_template.getPointCloud (), *transformed_cloud, best_alignment.final_transformation);
    ExtractAlignedModel(target_in,transformed_cloud,best_alignment,0.01f);

    std::stringstream ss;
    ss << "icp" << k << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *transformed_cloud, false); //*
    k++;


    if(fit_score <= fit_threshold)
    {
      try_times = 0;
/*
      pcl::PointCloud<pcl::PointXYZ>::Ptr icp_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      ICPCloud(transformed_cloud,target_in,icp_cloud,icp);
      icp_score = icp.getFitnessScore();
      std::cout << "icp_score = " << icp_score << std::endl;

      //WriteCloud(icp_cloud,"icpCloud.pcd");
      std::stringstream ss1;
      ss1 << "icp_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss1.str (), *icp_cloud, false);
*/
//      ExtractAlignedModel(target_in,transformed_cloud,best_alignment,0.01f);
      Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);
      PlateInformation oneInfor;
      oneInfor.x=translation[0];
      oneInfor.y=translation[1];
      oneInfor.z=translation[2];
      oneInfor.food = 0;
      getRadius(*best_template.getPointCloud (),oneInfor.radius);
      checkFood(target_in,oneInfor);

      infor.push_back(oneInfor);

      std::stringstream ss;
      ss << "model_aligned_" << j << ".pcd";
      writer.write<pcl::PointXYZ> (ss.str (), *transformed_cloud, false); //*
      printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
      std::cout<<"find a plate "<<std::endl;
      j++;

      std::cout << "--------------------" << std::endl;
      std::cout << fit_score << std::endl;
//        std::cout << icp_score << std::endl;
    }
    else
    {
      try_times += 1;
      std::cout<<"find a invalid target"<<std::endl;
    }
  }
  std::cout<< j << " plates have been found"<<std::endl;
  std::cerr << ">> Done: " << tt.toc () << " ms\n";
  return j;
}


void
ICPCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr target_in,pcl::PointCloud<pcl::PointXYZ>::Ptr model_in,pcl::PointCloud<pcl::PointXYZ>::Ptr& target_out,pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>& icp)
{
  pcl::console::TicToc tt;
  std::cerr << "ICP...\n", tt.tic ();
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



void
checkFood(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,PlateInformation& infor)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  PrePassThrough(cloud_in,cloud,infor.x-infor.radius,infor.x+infor.radius,"x");
  PrePassThrough(cloud,cloud,infor.y-infor.radius,infor.y+infor.radius,"y");
  PrePassThrough(cloud,cloud,0,infor.z,"z");
  ShowCloud(cloud);

  int num;
  num = ExtractClusters(cloud,0.005,10000,100);
  if(num > 0)
  {
    infor.food = 1;
  }
  else
  {
    infor.food = 0;
  }
}
