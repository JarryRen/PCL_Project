#include "pcl_control.h"

//test
#include <pcl/visualization/pcl_visualizer.h>

void scene::get_opt_string(int argc, char** argv)
{
  int opt_result;
  //可能需要的存储容器
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;
  scene::myfilter mf;
	
  while( ( opt_result = getopt(argc,argv,optString)) != -1 )
  {
    switch(opt_result)
    {
      case 'i':
	std::cout<<"------图像转点云."<<std::endl;
	scene::load_image(argc,argv,png_data);
	scene::image_to_pcd(png_data,cloud_data);
	scene::save_pcd(cloud_data);
	break;
      case 'l':
	scene::load_pcd(argc,argv,cloud_data);
	break;
      case 'c':
	std::cout<<"------ICP逐帧配准."<<std::endl;
	registration_test(cloud_data);
	break;
      case 'n':
	break;
      case 'r':
	std::cout<<"------表面重建."<<std::endl;
	mf.pcd_filter(cloud_data);
	reconstruction_test(cloud_data[0]);
	break;
      default:
	std::cout<<"Error Parameter!"<<std::endl;
	break;
    }
  }
}

void scene::load_file(int argc, char** argv)
{
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;
  scene::load_image(argc,argv,png_data);
  scene::image_to_pcd(png_data,cloud_data);
  scene::save_pcd(cloud_data);
}

void scene::registration_test(  std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data)
{
  scene::myfilter mf;
  mf.pcd_filter(cloud_data);
  
  pcl::PointCloud<PointT>::Ptr global_cloud (new pcl::PointCloud<PointT>);
  scene::registration reg;
  reg.global_registration(cloud_data,global_cloud);
  pcl::io::savePCDFile("global_cloud.pcd",*global_cloud,true);
}

void scene::reconstruction_test(pcl::PointCloud< scene::PointT >::Ptr cloud)
{
  
  pcl::PolygonMesh mesh;
  scene::reconsruction rec;
  rec.pcd_to_mesh(cloud,mesh,1);
  pcl::io::saveVTKFile("test_mesh.vtk",mesh);

  pcl::visualization::PCLVisualizer viewer("mesh viewer");
  viewer.addPolygonMesh(mesh,"mesh");
  while (!viewer.wasStopped())
   {
       viewer.spinOnce(100);
       boost::this_thread::sleep(boost::posix_time::microseconds(100000));
   }
  
}


