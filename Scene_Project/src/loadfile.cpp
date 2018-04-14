#include <loadfile.h>

void scene::load_image(int argc, char** argv, std::vector< cv::Mat >& png_data)
{
  std::string  extension(".png");
  for(int i=1;i<=argc/2;i++){//第一个参数为命令本身
    std::string filename=std::string(argv[i]);
    if(filename.size()<=extension.size())
      continue;
    cv::Mat rgb,depth;
    rgb=cv::imread(argv[i]);
    depth=cv::imread(argv[argc/2+i],-1);//onlyread
    png_data.push_back(rgb);
    png_data.push_back(depth);
  }
}

void scene::image_to_pcd(std::vector< cv::Mat >& png_data, std::vector< PointCloud >& cloud_data)
{
  cv::Mat rgb,depth;
  for(int i=0;i<png_data.size();i+=2)
  {
    rgb = png_data[i];
    depth = png_data[i+1];

    PointCloud::Ptr cloud(new PointCloud);
    for(int m=0;m<depth.rows;m++)
    {
      for(int n=0;n<depth.cols;n++)
      {
	ushort d = depth.ptr<ushort>(m)[n];
	if(d==0)
	  continue;
	PointT p;
	p.z=double(d) / camera_factor;
	p.x=(n-camera_cx)*p.z/camera_fx;
	p.y=(m-camera_cy)*p.z/camera_fy;
    
	p.b = rgb.ptr<uchar>(m)[n*3];
	p.g = rgb.ptr<uchar>(m)[n*3+1];
	p.r = rgb.ptr<uchar>(m)[n*3+2];
	
	cloud->points.push_back(p);
      }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense =false;
    cloud_data.push_back(*cloud);
  }
}

void scene::save_pcd(std::vector< PointCloud >& cloud_data)
{
  std::string  dir="./pcd_data";
  if(access(dir.c_str(),0)==-1)
  {
    std::cerr<<"pcd_data folder not existing!"<<std::endl;
    std::cerr<<"pcd_data make successfully."<<std::endl;
    mkdir(dir.c_str(),S_IRWXU);//创建文件夹
  }
  for(int i=0;i<cloud_data.size();i++)
  {
    PointCloud::Ptr cloud(new PointCloud);
    *cloud=cloud_data[i];
    std::stringstream ss;
    ss<<dir<<"/temp_pcd"<< i <<".pcd";
    pcl::io::savePCDFileASCII(ss.str(),*cloud);
    cloud->points.clear();
    std::cout<<"Point cloud "<< i <<"saved."<<std::endl;
  }
}

void scene::load_pcd(int argc, char** argv, std::vector< scene::PointCloud >& cloud_data)
{
  std::string  extension(".pcd");
  for(int i=1;i<=argc/2;i++){//第一个参数为命令本身
    std::string filename=std::string(argv[i]);
    if(filename.size()<=extension.size())
      continue;
    if (filename.compare (filename.size () - extension.size (), extension.size (), extension) == 0)//后缀检测是否为.pcd文件
    {
      PointCloud::Ptr cloud;
      pcl::io::loadPCDFile ( argv[i], *cloud );
      cloud_data.push_back ( *cloud );
    }
  }
}

