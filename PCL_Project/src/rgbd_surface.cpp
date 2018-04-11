#include <rgbd_surface.h>

void rgbd_pcl::rgbd_surface::mls_surface(pcl::PointCloud< pcl::PointXYZRGB >::Ptr input, pcl::PointCloud< pcl::PointXYZRGB >::Ptr output)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(input);
  
  pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
  mls.setInputCloud(input);
  mls.setComputeNormals(true);
  mls.setPolynomialFit(true);//启用多项式你拟合
  mls.setPolynomialOrder(3);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.04);//值越大，平滑范围越大，点越多
  /*
  mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::VOXEL_GRID_DILATION);
  mls.setDilationVoxelSize(1);
  mls.setDilationIterations(2);
  */
  mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::RANDOM_UNIFORM_DENSITY);
  mls.setPointDensity(100);
  
  mls.process(*output);
}

void rgbd_pcl::rgbd_surface::poisson_reconstruction(pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, pcl::PolygonMesh& mesh)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(30);
  ne.compute(*cloud_normals);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);
  
  pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
  poisson.setInputCloud(cloud_with_normals);

  poisson.setDepth(8);
  poisson.setSolverDivide(6);
  poisson.setIsoDivide(6);
  
  poisson.setConfidence(false);
  poisson.setManifold(false);
  poisson.setOutputPolygons(false);
  
  poisson.reconstruct(mesh);
}

void rgbd_pcl::rgbd_surface::gp3_reconstruction(pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, pcl::PolygonMesh& mesh)
{
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(30);
  ne.compute(*cloud_normals);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_normals(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree_normals->setInputCloud(cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree_normals);
  gp3.setSearchRadius (0.03);//最大边长,三角测量的最近邻居的球体半径
  gp3.setMu (3);//设置最近邻距离的乘数以获得每个点的最终搜索半径
  gp3.setMaximumNearestNeighbors (150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees 设置某点法线方向偏离样本点法线方向的最大角度
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(5*M_PI/6); // 150 degrees
  gp3.setNormalConsistency(false);//法向量是否连续变化
 
  gp3.reconstruct(mesh);
}
