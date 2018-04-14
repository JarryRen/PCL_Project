/**
 * 
 * 
 */
#pragma once

//pcl
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
//scene
#include "type.h"

namespace scene{ 
  class reconsruction{
    /** \brief 移动最小二乘 平滑处理
     * \param[in] input RGB点云
     * \param[out] output 平滑后RGB点云
     */ 
    void moving_least_sauares(pcl::PointCloud<PointT>::Ptr input,pcl::PointCloud<PointT>::Ptr output);
    
    void poisson(pcl::PointCloud<PointT>::Ptr input,pcl::PolygonMesh &mesh);
    
    /** \brief 贪婪三角投影
     * 
     * 
     */ 
    void greedy_projection_triangulation(pcl::PointCloud<PointT>::Ptr input,pcl::PolygonMesh &mesh);
  };
}