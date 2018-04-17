#include "reconstruction.h"

scene::reconsruction::reconsruction()
{
  get_config();
}

void scene::reconsruction::pcd_to_mesh(pcl::PointCloud< scene::PointT >::Ptr cloud, pcl::PolygonMesh& mesh, int method)
{
  pcl::PointCloud< scene::PointT >::Ptr cloud_mls(new pcl::PointCloud< scene::PointT >);
  moving_least_sauares(cloud,cloud_mls);
  if(method == 1){
    poisson(cloud_mls,mesh);
  }
  if(method == 2){
  }
}

void scene::reconsruction::moving_least_sauares(pcl::PointCloud< scene::PointT >::Ptr input, pcl::PointCloud< scene::PointT >::Ptr output)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(input);
  
  pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
  mls.setInputCloud(input);
  mls.setComputeNormals(true);
  mls.setPolynomialFit(true);//启用多项式你拟合
  mls.setPolynomialOrder(3);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(mls_radius);//值越大，平滑范围越大，点越多
  /*
  mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::VOXEL_GRID_DILATION);
  mls.setDilationVoxelSize(1);
  mls.setDilationIterations(2);
  */
  mls.process(*output);
}

void scene::reconsruction::poisson(pcl::PointCloud< scene::PointT >::Ptr cloud, pcl::PolygonMesh& mesh)
{
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(30);
  ne.compute(*cloud_normals);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);
  
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setInputCloud(cloud_with_normals);

  poisson.setDepth(depth);//
  poisson.setSolverDivide(solver_divide);
  poisson.setIsoDivide(iso_divide);
  
  poisson.setConfidence(false);
  poisson.setManifold(false);
  poisson.setOutputPolygons(false);
  
  poisson.reconstruct(mesh);
}

void scene::reconsruction::get_config()
{
    std::ifstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &rec = document["reconsruction"]; 
    mls_radius = rec["mls_radius"].GetFloat();
    
    depth= rec["depth"].GetInt();
    solver_divide = rec["solver_divide"].GetInt();
    iso_divide = rec["iso_divide"].GetInt();
}
