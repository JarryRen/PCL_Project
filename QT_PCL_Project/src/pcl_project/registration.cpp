#include "pcl_project/registration.h"

gp::registration::registration()
{
  get_config();
}

void gp::registration::run(pcl::PointCloud<PointT>::Ptr source, pcl::PointCloud<PointT>::Ptr target, Eigen::Matrix4f &complete_trans)
{
    Eigen::Matrix4f sac_trans;
    pcl::PointCloud<PointT>::Ptr source_sac(new pcl::PointCloud<PointT>);
    sac_ia(source, target, source_sac, sac_trans);

    iterative_closest_point(source_sac, target, sac_trans, complete_trans);
}

void gp::registration::pre_process(pcl::PointCloud< PointT >::Ptr input, pcl::PointCloud< PointT >::Ptr output, pcl::PointCloud< pcl::FPFHSignature33 >::Ptr fpfhs)
{
  //去除NAN点
  std::vector<int> indieces;
  pcl::removeNaNFromPointCloud(*input,*input,indieces);

  pcl::VoxelGrid<PointT> grid;
  grid.setInputCloud(input);
  grid.setLeafSize(leaf_size,leaf_size,leaf_size);
  grid.filter(*output);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal> );
  pcl::search::KdTree<PointT>::Ptr tree_normals(new pcl::search::KdTree<PointT> );
  tree_normals->setInputCloud(output);
  pcl::NormalEstimation<PointT,pcl::Normal> ne;
  ne.setInputCloud(output);
  ne.setRadiusSearch(ne_radius);
  ne.setSearchMethod(tree_normals);
  ne.compute(*cloud_normals);
  
  pcl::search::KdTree<PointT>::Ptr tree_fpfh(new pcl::search::KdTree<PointT> );
  tree_fpfh->setInputCloud(output);
  pcl::FPFHEstimationOMP<PointT,pcl::Normal,pcl::FPFHSignature33> fpfh;
  //4线程
  fpfh.setNumberOfThreads(4);
  fpfh.setInputCloud(output);
  fpfh.setInputNormals(cloud_normals);
  fpfh.setSearchMethod(tree_fpfh);
  fpfh.setRadiusSearch(fpfh_radius);
  fpfh.compute(*fpfhs);
}

void gp::registration::sac_ia(pcl::PointCloud< PointT >::Ptr source, pcl::PointCloud< PointT >::Ptr target,
                 pcl::PointCloud< PointT >::Ptr source_out,Eigen::Matrix4f &sac_trans)
{
  pcl::PointCloud< PointT >::Ptr target_o(new pcl::PointCloud< PointT > );
  pcl::PointCloud< pcl::FPFHSignature33 >::Ptr source_fpfhs(new pcl::PointCloud< pcl::FPFHSignature33 >);
  pcl::PointCloud< pcl::FPFHSignature33 >::Ptr target_fpfhs(new pcl::PointCloud< pcl::FPFHSignature33 >);

  pre_process(source,source_out,source_fpfhs);
  pre_process(target,target_o,target_fpfhs);
  
  pcl::PointCloud<PointT>::Ptr sac_align(new pcl::PointCloud<PointT>);
  pcl::SampleConsensusInitialAlignment<PointT,PointT,pcl::FPFHSignature33> sac_ia;
  sac_ia.setInputSource(source_out);
  sac_ia.setSourceFeatures(source_fpfhs);
  sac_ia.setInputTarget(target_o);
  sac_ia.setTargetFeatures(target_fpfhs);
  sac_ia.align(*sac_align);
  sac_trans = sac_ia.getFinalTransformation();
}

void gp::registration::iterative_closest_point(pcl::PointCloud< PointT >::Ptr source, pcl::PointCloud< PointT >::Ptr target,
						  Eigen::Matrix4f& sac_trans, Eigen::Matrix4f& icp_trans)
{
  pcl::PointCloud<PointT>::Ptr icp_cloud(new pcl::PointCloud<PointT>);
  pcl::IterativeClosestPoint<PointT,PointT> icp;
  icp.setInputSource(source);//处理后的源点云
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(max_distance);
  icp.setMaximumIterations(max_iterations);
  icp.setTransformationEpsilon(trans_epsilon);
  icp.setEuclideanFitnessEpsilon(ef_epsilon);
  icp.align(*icp_cloud,sac_trans);
  
  icp_trans = icp.getFinalTransformation();
}

void gp::registration::global_registration(std::vector< boost::shared_ptr< pcl::PointCloud< PointT > > >& cloud_data,
                          pcl::PointCloud< PointT >::Ptr global_alian_cloud)
{
  if( cloud_data.empty() )
  {
    PCL_ERROR("cloud_data is empty in gp::registration::global_registration.");
  }
  pcl::PointCloud<PointT>::Ptr icp_cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr source,target;

  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
  Eigen::Matrix4f sac_trans,icp_trans;
  
  *global_alian_cloud = *cloud_data[0];
  
  for(int i = 1;i<cloud_data.size();i++){
    source = cloud_data[i];
    target = cloud_data[i-1];
    std::cout<<"第"<<i<<"次配准"<<std::endl;
    
    pcl::PointCloud<PointT>::Ptr source_process(new pcl::PointCloud<PointT>);
    sac_ia(source,target,source_process,sac_trans);
    std::cout<<"处理后源点云:"<<source_process->size()<<" 目标点云:"<<target->size()<<std::endl;
    std::cout<<"------sac_ia 变换矩阵: "<<std::endl;
    std::cout<<sac_trans<<std::endl;
    
    iterative_closest_point(source_process,target,sac_trans,icp_trans);
    std::cout<<"------icp 变换矩阵: "<<std::endl;
    std::cout<<icp_trans<<std::endl;
    
    //使用创建的变换对未过滤的输入点云进行变换
    pcl::transformPointCloud(*source, *icp_cloud, icp_trans);
    //对变换后的源点云进行全局变换
    pcl::transformPointCloud(*icp_cloud,*icp_cloud,GlobalTransform);
    *global_alian_cloud += *icp_cloud;
    //更新全局变换
    GlobalTransform = GlobalTransform * icp_trans;
  }
}

void gp::registration::get_config()
{
    std::ifstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &reg = document["registration"]; 
    leaf_size = reg["leaf_size"].GetFloat();
    ne_radius = reg["ne_radius"].GetDouble();
    fpfh_radius = reg["fpfh_radius"].GetDouble();
    
    max_distance = reg["max_distance"].GetDouble();
    max_iterations = reg["max_iterations"].GetInt();
    trans_epsilon = reg["trans_epsilon"].GetDouble();
    ef_epsilon = reg["ef_epsilon"].GetDouble();
}


