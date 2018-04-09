/** \brief  点云表面重建
 * \date 18/4/8
 * 
 */
#pragma once
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

namespace rgbd_pcl{
  class rgbd_surface{
  public:
    
    /** \brief 最小二乘法 平滑处理
     * \param[in] input RGB点云
     * \param[out] output 平滑后RGB点云
     */ 
    void rgbd_mls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
		  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output
    );
    
    /** \brief 泊松重建
     * 
     */ 
    void rgbd_poisson();
    
  private:
    
    
    
  };
 
}