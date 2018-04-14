#include <pcl_control.h>

void scene::load_file(int argc, char** argv)
{
  std::vector<cv::Mat> png_data;
  std::vector<PointCloud > cloud_data;
  scene::load_image(argc,argv,png_data);
  scene::image_to_pcd(png_data,cloud_data);
  scene::save_pcd(cloud_data);
}
