
#pragma once
//linux io 
#include <unistd.h>
//scene
#include "loadfile.h"
#include "registration.h"
#include "reconstruction.h"

namespace scene{
  
  /** \brief 选项字符串，命令行参数设置
   * \'i' 图像转PCD
   * \'p' load pcd
   * \'c' icp 
   * \'r'  reconstruction
   */ 
  static const char *optString = "ipcr";  
  
  void get_opt_string(int argc,char **argv);
  
  void load_file(int argc,char **argv);
  
  void registration_test(  std::vector<pcl::PointCloud<PointT>::Ptr > cloud_data);
}