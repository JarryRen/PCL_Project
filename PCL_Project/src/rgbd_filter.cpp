#include <rgbd_filter.h>

void rgbd_pcl::rgb_filter::f_pass(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output)
{
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(input);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.5,1.5);
  pass.setFilterLimitsNegative(false);//保留点内
  pass.filter(*output);
}

void rgbd_pcl::rgb_filter::f_voxel_grid(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output)
{
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setInputCloud(input);
  grid.setLeafSize(0.001f,0.001f,0.001f);
  grid.setDownsampleAllData(true);
  grid.filter(*output);
}
