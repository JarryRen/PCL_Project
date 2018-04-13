/** \brief 点云配准
 * SAC_IA 粗配准 + icp精配准 
 * \date 18/4/7
 * 
 */

#pragma once

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <rgbd_features.h>
namespace rgbd_pcl{
  class rgbd_reg{
  public:
    
    /** \brief
     * 
     * 
     */ 
    void 
    sac_align(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_f, 
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_f,
	      pcl::FPFHEstimation<pcl::PointXYZRGB,pcl::Normal,pcl::FPFHSignature33> &fpfhs_src, 
	      pcl::FPFHEstimation<pcl::PointXYZRGB,pcl::Normal,pcl::FPFHSignature33> &fpfhs_tgt,
	      Eigen::Matrix4f &sac_trans
 	    );
    
    
    void
    icp_alian(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src_f,
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt,
	      Eigen::Matrix4f &sac_trans,
	      pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_result
    );
    
  };
  
}