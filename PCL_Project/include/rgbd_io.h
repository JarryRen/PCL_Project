/** \brief 文件读写相关操作
 * \date 18/4/7
 * 
 * 
 */
#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace rgbd_pcl{
  class rgbd_io{
  public:
    
    /**
     * \param[in] argc 执行程序输入指令长 
     * \param[in] argv 具体参数组
     * \param[out] vector_cloud rgb点云容器 
     */ 
    void load_pcd_file(char argc,char **argv,std::vector<pcl::PointXYZ> &vector_cloud);
    
    /**
     * \param[in] argc 执行程序输入指令长 
     * \param[in] argv 具体参数组
     * \param[out] png_data rgb+depth图像容器
     */ 
    void load_png_file(int argc,char **argv,std::vector<cv::Mat> &png_data);
    
    /**
     * \param[out] 
     * 
     */ 
    void image_to_pcd();
    
    //相机内参
  private:
    static const double camera_factor;
    static const double camera_cx;
    static const double camera_cy;
    static const double camera_fx;
    static const double camera_fy;

  };
  //相机内参
  const double rgbd_io::camera_factor=1000;
  const double rgbd_io::camera_cx = 325.5;
  const double rgbd_io::camera_cy = 253.5;
  const double rgbd_io::camera_fx = 518.0;
  const double rgbd_io::camera_fy = 519.0;
   
}