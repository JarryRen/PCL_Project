#include "pcl_project/pcl_control.h"

//test
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>

void gp::get_opt_string(int argc, char** argv)
{
  int opt_result;
  //可能需要的存储容器
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;
  gp::myfilter mf;
  
  while( ( opt_result = getopt(argc,argv,optString)) != -1 )
  {
    switch(opt_result)
    {
      case 'i':
	std::cout<<"------图像转点云."<<std::endl;
	gp::load_image(argc,argv,png_data);
	gp::image_to_pcd(png_data,cloud_data);
	gp::save_pcd(cloud_data);
	break;
      case 'l':
	gp::load_pcd(argc,argv,cloud_data);
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
      case 't':
	std::cout<<"//test."<<std::endl;
	test();
	break;
      default:
	std::cout<<"Error Parameter!"<<std::endl;
	std::cout<<"-i 图像转PCD"<<std::endl;
	std::cout<<"-l load pcd"<<std::endl;
	std::cout<<"-t icp "<<std::endl;
	std::cout<<"-n ndt"<<std::endl;
	std::cout<<"-r 重建"<<std::endl;
	std::cout<<"-t test"<<std::endl;
	break;
    }
  }
}

void gp::load_file(int argc, char** argv)
{
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;
  gp::load_image(argc,argv,png_data);
  gp::image_to_pcd(png_data,cloud_data);
  gp::save_pcd(cloud_data);
}

void gp::registration_test(  std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data)
{
  gp::myfilter mf;
  mf.pcd_filter(cloud_data);
  
  pcl::PointCloud<PointT>::Ptr global_cloud (new pcl::PointCloud<PointT>);
  gp::registration reg;
  reg.global_registration(cloud_data,global_cloud);
  pcl::io::savePCDFile("global_cloud.pcd",*global_cloud,true);
}

void gp::reconstruction_test(pcl::PointCloud< gp::PointT >::Ptr cloud)
{
  pcl::PolygonMesh mesh;
  gp::reconsruction rec;
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

void gp::test()
{
  pcl::visualization::PCLVisualizer viewer("mesh viewer");
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFileVTK("test_mesh.vtk",mesh);
  viewer.setBackgroundColor(128,128,128);
  viewer.addPolygonMesh(mesh,"mesh");
  while (!viewer.wasStopped())
  {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

