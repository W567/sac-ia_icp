#include "cook_io.h"

void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDReader reader;
  reader.read (name,*cloud_in);
  std::cerr << "PointCloud has: " << cloud_in->points.size () << " data points." << std::endl;
}

void ReadCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name,int num, std::string ext)
{
  std::ostringstream sstr;
  sstr << num;
  std::string str = sstr.str();
  std::string final_name = name + str + ext;
  pcl::PCDReader reader;
  reader.read (final_name,*cloud_in);
  std::cerr << "PointCloud has: " << cloud_in->points.size () << " data points." << std::endl;
}

void WriteCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,std::string name)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> (name, *cloud_in);
}

void ShowCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,int coor_size,int R,int G,int B)
{
  if (cloud_in->points.empty())
    std::cerr << " No points exist." << std::endl;
  else
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Vierwer"));
    viewer->setBackgroundColor(R,G,B);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_in,"result");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"result");
    viewer->addCoordinateSystem(coor_size);
    viewer->initCameraParameters();

    while(!viewer->wasStopped())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
  }
}

int
loadCloud(int argc,char** argv,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if( argc < 2)
    {
        printf("No target Cloud files given!\n");
    }

    std::vector<int> filenames;
    bool file_is_pcd = false;

    filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");

    if (filenames.size () != 1)  {
      filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

      if (filenames.size () != 1) {
        printf("input error!\n");
        return -1;
      } else {
        file_is_pcd = true;
      }
    }

    if (file_is_pcd) {
      if (pcl::io::loadPCDFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    } else {
      if (pcl::io::loadPLYFile (argv[filenames[0]], *cloud) < 0)  {
        std::cout << "Error loading point cloud " << argv[filenames[0]] << std::endl << std::endl;
        return -1;
      }
    }
    return 0;
}
