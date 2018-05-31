/**
 * 
 * 
 */
#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/gp3.h>
//rapidjson
#include "3rdParty/rapidjson/document.h"
#include "3rdParty/rapidjson/istreamwrapper.h"
//gp

namespace gp{ 

    /**
    * @brief operator * 法向量角度计算
    * @param p1
    * @param p2
    * @return
    */
    template<typename PointT>
    float operator *(PointT &p1,PointT &p2)
    {
        float cost;
        cost = p1.normal_x * p2.normal_y +
                p1.normal_y * p2.normal_y +
                p1.normal_z * p2.normal_z;
        return cost;
    }

    /**
   * @brief The reconsruction class 模型重建类
   */
  class reconsruction{
      typedef pcl::PointXYZRGB PointT;
  public:
    reconsruction();
    
    /**
     * @brief run Qt界面使用的函数
     * @param cloud
     * @param mesh
     */
    void run(pcl::PointCloud< PointT >::Ptr cloud, pcl::PolygonMesh& mesh);

    /** \brief 点云转换到模型  命令行情况
     * \param[in] cloud 输入点云
     * \param[out] mesh 重建后的网格模型
     * \param[in] method 使用方法 \1 泊松  \2 三角化
     */ 
    void pcd_to_mesh(pcl::PointCloud< PointT >::Ptr cloud, pcl::PolygonMesh& mesh, int method);

  protected:
    /**
     * @brief 移动最小二乘 平滑处理
     * @param[in] input RGB点云
     * @param[out] output 平滑后RGB点云
     */ 
    void movingLeastSquares(pcl::PointCloud<PointT>::Ptr cloud,pcl::PointCloud<PointT>::Ptr cloud_mls);
    
    /**
     * @brief normlasConsistency 法向量一致化
     * @param cloud
     * @param cloud_with_normals
     */
    void normlasConsistency(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals);
    
    /**
     * @brief 泊松重建
     * @param[in] cloud_with_normals 输入点云及法向量
     * @param[out] mesh 重建后的网格模型
     */ 
    void poisson(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals,pcl::PolygonMesh &mesh);

    /**
     * @brief removeRedundantPolygon 去除冗余曲面
     * @param[in] cloud_mls 被平滑的点云，poisson重建的源点云
     * @param[out] mesh 去除的网格模型
     */
    void removeRedundantPolygon(pcl::PointCloud<PointT>::Ptr cloud_mls, pcl::PolygonMesh &mesh);
    
    /**
     * @brief greedy_projection_triangulation
     * @param cloud
     * @param mesh
     */
    void greedy_projection_triangulation(pcl::PointCloud<PointT>::Ptr cloud,pcl::PolygonMesh &mesh);
    
    /**
     * @brief 模型染色
     * @param mesh 处理的模型
     * @param cloud 处理前点云数据，用于颜色提取
     */ 
    void meshColoring(pcl::PolygonMesh &mesh,pcl::PointCloud<PointT>::Ptr cloud);

  private:
    float leaf_size;
    double mls_radius;
    //poisson param
    int depth;
    int solver_divide;
    int iso_divide;

    int max_polygon;//选取排名前max_polygon的三角面片类型

    /**
     * @brief 取得配置信息，文件为当前目录
     * @index 文件 project_config.json
     */
    void get_config();
  };

}
