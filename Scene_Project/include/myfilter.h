#pragma once
//pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
//rapidjson
#include "3rdParty/rapidjson/document.h"
#include "3rdParty/rapidjson/istreamwrapper.h"
#include <fstream>
//scene
#include "type.h"

namespace scene {
  
  class myfilter{
  public:
    myfilter();
    
    /** \brief 体素滤波及去除离群点
     * 
     */ 
    void pcd_filter(std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data);
    
  private:
    double leaf_size;
    int nr_k;
    double stddev_mult;
    
    /** \brief 取得配置信息，文件为当前目录
     * \in 文件 project_config.json
     */ 
    void get_config();
  };
  
}