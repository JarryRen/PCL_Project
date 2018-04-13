/** \brief 特征计算
 * \date 18/4/7
 */ 
#pragma once
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

namespace rgbd_pcl{
  class rgbd_features{
  public:
    
    /** \brief 估计法线
     * \param[in] input RGB点云
     * \param[out] output 法线点云
     */ 
    void estimation_normals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
			pcl::PointCloud<pcl::Normal>::Ptr output
    );
    
    /** \brief 估计FPFH
     * \param[in] input_cloud RGB点云
     * \param[in] input_normals 点云法线
     * \param[out] output 快速点特征直方图(FPFH)描述子
     */ 
    void estimation_fpfh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud,
			 pcl::PointCloud<pcl::Normal>::Ptr input_normals,
			 pcl::PointCloud<pcl::FPFHSignature33>::Ptr output
    );
    
  private:
    int normals_redius;
    int fpfh_redius;
  };
  
}