/** \brief
 * 
 * 
 */ 

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>

#include <pcl/search/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>  
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>  
#include <pcl/surface/poisson.h>  
#include <pcl/filters/radius_outlier_removal.h>

#include <boost/thread/thread.hpp>  
#include <boost/graph/graph_concepts.hpp>


void gp3_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh &mesh){
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
  ne.compute(*cloud_normals);
  
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::concatenateFields(*cloud,*cloud_normals,*cloud_with_normals);

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree_normals(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  tree_normals->setInputCloud(cloud_with_normals);

  pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree_normals);
  gp3.setSearchRadius (0.01);
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (50);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  
  gp3.reconstruct(mesh);
  
}

void poisson_reconstruction(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PolygonMesh &mesh){
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);
  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
  ne.setInputCloud(cloud);
  ne.setSearchMethod(tree);
  ne.setKSearch(20);
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

void mls_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr output){
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(input);
  
  pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
  mls.setInputCloud(input);
  mls.setComputeNormals(true);
  mls.setPolynomialFit(true);//启用多项式你拟合
  mls.setPolynomialOrder(3);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.02);//值越大，平滑范围越大，点越多
  
  /*
  mls.setUpsamplingMethod(pcl::MovingLeastSquares< pcl::PointXYZRGB, pcl::PointXYZRGB >::VOXEL_GRID_DILATION);
  mls.setDilationVoxelSize(1);
  mls.setDilationIterations(2);
  */
  
  mls.process(*output);
}

int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile("test.pcd",*cloud_in)==-1){
      PCL_ERROR("no data!");
      return -1;
    };
    
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize(0.005f,0.005f,0.005f);
    grid.setDownsampleAllData(true);
    grid.setInputCloud(cloud_in);
    grid.filter(*cloud_in);
    
    
    
    
    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
    ror.setRadiusSearch(0.01);
    ror.setMinNeighborsInRadius(10);
    ror.setInputCloud(cloud_in);
    ror.filter(*cloud_in);
    
    //mls
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    mls_surface(cloud_in,cloud);
    
    //poisson gp3
 
    pcl::PolygonMesh mesh;
    gp3_reconstruction(cloud,mesh);
    
    pcl::io::saveVTKFile("test_mesh.vtk",mesh);
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D viewer"));  
    viewer->setBackgroundColor(0, 0, 0);  
    viewer->addPolygonMesh(mesh, "my");  
  //  viewer->addCoordinateSystem(1.0);  
  //  viewer->initCameraParameters();  
    while (!viewer->wasStopped()){  
	viewer->spinOnce(100);  
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));  
    }  

    return 0;
}