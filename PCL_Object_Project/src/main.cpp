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
#include <pcl/filters/statistical_outlier_removal.h>

#include <boost/thread/thread.hpp>  
#include <boost/graph/graph_concepts.hpp>

#include <rgbd_surface.h>


int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    if(pcl::io::loadPCDFile(argv[1],*cloud_in)==-1){
      PCL_ERROR("no data!");
      return -1;
    };
    
    
    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize(0.003f,0.003f,0.003f);
    grid.setDownsampleAllData(false);
    grid.setInputCloud(cloud_in);
    grid.filter(*cloud_in);

    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setMeanK(20);
    sor.setStddevMulThresh(0.7);//离群点标准差     
    sor.setInputCloud(cloud_in);
    sor.filter(*cloud_in);
    
    
    
    rgbd_pcl::rgbd_surface surface;
    
    //mls
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    surface.mls_surface(cloud_in,cloud);
    
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_cloud(new pcl::visualization::PCLVisualizer("3D viewer"));  
  viewer_cloud->addPointCloud(cloud,"cloud");
    while (!viewer_cloud->wasStopped()){  
	viewer_cloud->spinOnce(100);  
	boost::this_thread::sleep(boost::posix_time::microseconds(100000));  
    }
    
    //poisson gp3
 
    pcl::PolygonMesh mesh;
    surface.poisson_reconstruction(cloud,mesh);
    
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