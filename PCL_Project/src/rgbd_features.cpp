#include <rgbd_features.h>

void rgbd_pcl::rgbd_features::estimation_normals(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::Normal >::Ptr output)
{
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  ne.setInputCloud(input);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(0.015);
  ne.compute(*output);
}

void rgbd_pcl::rgbd_features::estimation_fpfh(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input_cloud, pcl::PointCloud< pcl::Normal >::Ptr input_normals, pcl::PointCloud< pcl::FPFHSignature33 >::Ptr output)
{
   pcl::FPFHEstimation<pcl::PointXYZRGB,pcl::Normal,pcl::FPFHSignature33> fpfh;
   pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
   fpfh.setInputCloud(input_cloud);
   fpfh.setInputNormals(input_normals);
   fpfh.setSearchMethod(tree);
   fpfh.setRadiusSearch(0.03);
   fpfh.compute(*output);
}

