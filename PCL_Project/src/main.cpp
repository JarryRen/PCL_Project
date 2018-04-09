/** 
 * 
 * 
 */ 

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rgbd_surface.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>

#include <rgbd_filter.h>

int main(int argc, char **argv) {
    std::cout << "Hello, world!" << std::endl;
  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("test.pcd",*cloud);
    
    
    
    return 0;
}