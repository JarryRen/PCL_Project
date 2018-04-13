/** \brief  点云表面重建
 * \date 18/4/8
 * 
 */
#pragma once
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>

namespace rgbd_pcl{
  class rgbd_surface{
  public:
    
    /** \brief 最小二乘法 平滑处理
     * \param[in] input RGB点云
     * \param[out] output 平滑后RGB点云
     */ 
    void mls_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    
    /** \brief 泊松重建
     * 
     */ 
    void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh &mesh);
    
    /** \brief 贪婪三角化
     * 
     * 
     */ 
    void gp3_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh &mesh);
 
  private:
    
    
    
  };
 
}