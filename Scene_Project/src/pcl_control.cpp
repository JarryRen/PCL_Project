#include <pcl_control.h>

void scene::get_opt_string(int argc, char** argv)
{
  //可能需要的存储容器
  std::vector<cv::Mat> png_data;
  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data;
  
    int opt_result;
  
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
