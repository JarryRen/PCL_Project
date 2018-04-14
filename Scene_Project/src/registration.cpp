#include "registration.h"

void scene::registration::pre_process(pcl::PointCloud<PointT>::Ptr input, pcl::PointCloud<PointT>::Ptr output, pcl::PointCloud< pcl::FPFHSignature33 >::Ptr fpfhs)
{
  //去除 NAN 点
  std::vector<int> indices; 
  pcl::removeNaNFromPointCloud(*input, *input, indices);
  
  pcl::VoxelGrid<PointT> grid;
  grid.setLeafSize(leaf_size,leaf_size,leaf_size);
}



