
#pragma once
//linux io 
#include <unistd.h>
//scene
#include "loadfile.h"
#include "registration.h"
#include "reconstruction.h"
#include "myfilter.h"

namespace scene{
  
  /** \brief 选项字符串，命令行参数设置
   * \'i' 图像转PCD
   * \'l' load pcd
   * \'c' icp 
   * \'n' ndt
   * \'r' 重建 reconstruction
   * \'t' 测试
   */ 
  static const char *optString = "ilcnrt";  
  
  void get_opt_string(int argc,char **argv);
  
  void load_file(int argc,char **argv);
  
  void registration_test(  std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data);
  
  void reconstruction_test(pcl::PointCloud<PointT>::Ptr cloud);
  
  void test();
}