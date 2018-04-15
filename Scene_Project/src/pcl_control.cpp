#include <pcl_control.h>

void scene::get_opt_string(int argc, char** argv)
{
  int opt_result;
  //可能需要的存储容器
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;

  while( ( opt_result = getopt(argc,argv,optString)) != -1 )
  {
    switch(opt_result)
    {
      case 'i':
	scene::load_image(argc,argv,png_data);
	scene::image_to_pcd(png_data,cloud_data);
	scene::save_pcd(cloud_data);
	break;
      case 'p':
	scene::load_pcd(argc,argv,cloud_data);
	break;
      case 'c':
	registration_test(cloud_data);
	break;
      case 'r':
	
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

void scene::registration_test(  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data)
{
    pcl::PointCloud<PointT>::Ptr global_cloud (new pcl::PointCloud<PointT>);
    scene::registration reg(1);
//    reg.global_registration(cloud_data,global_cloud);
//    pcl::io::savePCDFile("global_cloud.pcd",*global_cloud,true);
}
