
#pragma once
//linux io 
#include <unistd.h>
//gp
#include "pcl_project/loadfile.h"
#include "pcl_project/registration.h"
#include "pcl_project/reconstruction.h"
#include "pcl_project/myfilter.h"

namespace gp{
  
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