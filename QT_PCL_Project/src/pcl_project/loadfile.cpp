#include "pcl_project/loadfile.h"

void gp::load_image(int argc, char** argv, std::vector< cv::Mat >& png_data)
{
    std::string  extension(".png");
  //第一个参数为命令本身,最后为选项参数
    for(int i=1;i<=(argc-1)/2;i++){
        std::string filename=std::string(argv[i]);
        if(filename.size()<=extension.size())
            continue;
        cv::Mat rgb,depth;
        rgb=cv::imread(argv[i]);
        depth=cv::imread(argv[(argc-1)/2+i],-1);//onlyread
        png_data.push_back(rgb);
        png_data.push_back(depth);
    }
}

void gp::image_to_pcd(std::vector< cv::Mat >& png_data, std::vector< pcl::PointCloud<PointT>::Ptr >& cloud_data)
{
    //KinectV1相机内参
    const double CAMERA_FACTOR = 1000;
    const double CAMERA_CX = 325.5;
    const double CAMERA_CY = 253.5;
    const double CAMERA_FX = 518.0;
    const double CAMERA_FY = 519.0;



  cv::Mat rgb,depth;
  for(int i=0;i<png_data.size();i+=2)
  {
    rgb = png_data[i];
    depth = png_data[i+1];

    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    for(int m=0;m<depth.rows;m++)
    {
      for(int n=0;n<depth.cols;n++)
      {
	ushort d = depth.ptr<ushort>(m)[n];
	if(d==0)
	  continue;
	PointT p;
    p.z=double(d) / CAMERA_FACTOR;
    p.x=(n-CAMERA_CX)*p.z/CAMERA_FX;
    p.y=(m-CAMERA_CY)*p.z/CAMERA_FY;
    
	p.b = rgb.ptr<uchar>(m)[n*3];
	p.g = rgb.ptr<uchar>(m)[n*3+1];
	p.r = rgb.ptr<uchar>(m)[n*3+2];
	
	cloud->points.push_back(p);
      }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense =false;
    cloud_data.push_back(cloud);
  }
}

void gp::save_pcd(std::vector< pcl::PointCloud<PointT>::Ptr >& cloud_data)
{
  std::string  dir="./pcd_data";
  if(access(dir.c_str(),0)==-1)
  {
    std::cerr<<"pcd_data folder not existing!"<<std::endl;
    std::cerr<<"pcd_data make successfully."<<std::endl;
    //创建文件夹
    mkdir(dir.c_str(),S_IRWXU);
  }
  for(int i=0;i<cloud_data.size();i++)
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    cloud=cloud_data[i];
    std::stringstream ss;
    ss<<dir<<"/temp_pcd"<< i <<".pcd";
    pcl::io::savePCDFileASCII(ss.str(),*cloud);
    cloud->points.clear();
    std::cout<<"Point cloud "<< i+1 <<"saved."<<std::endl;
  }
}

void gp::load_pcd(int argc, char** argv, std::vector< pcl::PointCloud<PointT>::Ptr >& cloud_data)
{
  std::cout<<"Point Cloud loading..."<<std::endl;
  std::string  extension(".pcd");
  for(int i=1;i<argc;i++){
    std::string filename=std::string(argv[i]);
    if(filename.size()<=extension.size())
      continue;
    //比较，检测后缀是否为.pcd文件
    if (filename.compare (filename.size () - extension.size (), extension.size (), extension) == 0)
    {
      std::cout<<"Point Cloud "<<argv[i]<<" loaded."<<std::endl;
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      pcl::io::loadPCDFile ( argv[i], *cloud );
      cloud_data.push_back ( cloud );
    }
  }
}

