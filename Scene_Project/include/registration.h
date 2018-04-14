
#pragma once

//pcl
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <boost/graph/graph_concepts.hpp>
//scene
#include "type.h"

namespace scene{
  class registration{
  public:
    /** \brief 全局配准
     * \param[in] cloud_data 点云对象容器
     * \param[out] cloud 配准完成的点云对象指针
     */ 
    void global_registration(std::vector<pcl::PointCloud<PointT>::Ptr > &cloud_data,pcl::PointCloud<PointT>::Ptr cloud);
  
  private:
    int leaf_size;
    int ne_radius;//法线计算半径
    int fpfh_radius;//fpfh特征子计算半径
    
    int max_distance;//MaxCorrespondenceDistance
    int max_iterations;//MaximumIterations
    int trans_epsilon;//TransformationEpsilon
    int ef_epsilon;//EuclideanFitnessEpsilon
    
    /** \brief 预处理，过滤，提取特征
     * \param[in] input 待处理的源点云
     * \param[out] output 处理后点云
     * \param[out] fpfhs FPFH特征子数据
     */ 
    void pre_process(pcl::PointCloud<PointT>::Ptr input,pcl::PointCloud<PointT>::Ptr output,
		     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs
    );
    
    /** \brief 迭代最近点
     * \param[in] cloud_src 待转换的源点云
     * \param[in] cloud_tgt 目标点云
     * \param[out] icp_trans 转换矩阵
     */ 
    void iterative_closest_point(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_tgt, Eigen::Matrix4f &icp_trans);
  };
  
}