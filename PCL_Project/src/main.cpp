/** 
 * 
 * 
 */ 

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/search/kdtree.h>

#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>  
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/gp3.h>  
#include <pcl/surface/poisson.h>  

#include <boost/thread/thread.hpp>  



int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile("test.pcd",*cloud)==-1){
      PCL_ERROR("no data!");
      return -1;
    };
    
   
    //filters
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize(0.005f,0.005f,0.005f);
    grid.setInputCloud(cloud);
    grid.filter(*cloud);
    
    //mls
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_mls(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::MovingLeastSquares<pcl::PointXYZRGB,pcl::PointXYZRGB> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree_mls);
    mls.setSearchRadius(0.02);
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
	
    mls.process(*temp);

    //poisson
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(20);
    ne.compute(*normals);
    
    pcl::concatenateFields(*cloud,*normals,*cloud_normals);
  
    //创建搜索树  
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);  
    tree2->setInputCloud(cloud_normals);  
    //创建Poisson对象，并设置参数  
    pcl::Poisson<pcl::PointXYZRGBNormal> pn;  
    pn.setConfidence(true); //是否使用法向量的大小作为置信信息。如果false，所有法向量均归一化。  
    pn.setDegree(2); //设置参数degree[1,5],值越大越精细，耗时越久。  
    pn.setDepth(8); //树的最大深度，求解2^d x 2^d x 2^d立方体元。由于八叉树自适应采样密度，指定值仅为最大深度。  
    pn.setIsoDivide(8); //用于提取ISO等值面的算法的深度  
    pn.setManifold(true); //是否添加多边形的重心，当多边形三角化时。 设置流行标志，如果设置为true，则对多边形进行细分三角话时添加重心，设置false则不添加  
    pn.setOutputPolygons(false); //是否输出多边形网格（而不是三角化移动立方体的结果）  
    pn.setSamplesPerNode(3.0); //设置落入一个八叉树结点中的样本点的最小数量。无噪声，[1.0-5.0],有噪声[15.-20.]平滑  
    pn.setScale(1.25); //设置用于重构的立方体直径和样本边界立方体直径的比率。  
    pn.setSolverDivide(8); //设置求解线性方程组的Gauss-Seidel迭代方法的深度  
    //pn.setIndices();  
      
	    //设置搜索方法和输入点云  
    pn.setSearchMethod(tree2);  
    pn.setInputCloud(cloud_normals);  
    //创建多变形网格，用于存储结果  
	
    pcl::PolygonMesh mesh;
    pn.performReconstruction(mesh);
    
    
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