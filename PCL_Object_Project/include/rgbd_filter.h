/** \brief 对点云进行各式滤波
 * 
 * \date 18/4/7
 */ 
#pragma once
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

namespace rgbd_pcl{
  class rgb_filter{
  public:
    rgb_filter();
    
    void f_pass(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
		      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    
    void f_voxel_grid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
		      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output
    );
    
  private:
    std::string field_name;
    float limit_min;
    float limit_max;
    int leaf_size;
    
  };
}