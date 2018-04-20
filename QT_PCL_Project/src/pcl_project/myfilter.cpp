#include "pcl_project/myfilter.h"

gp::myfilter::myfilter()
{
  get_config();
}

void gp::myfilter::pcd_filter(std::vector< boost::shared_ptr< pcl::PointCloud< gp::PointT > > >& cloud_data)
{
  for(int i=0; i<cloud_data.size();i++)
  {
    std::vector<int> indieces;
    pcl::removeNaNFromPointCloud(*cloud_data[i],*cloud_data[i],indieces);
  
    pcl::PointCloud<PointT>::Ptr cloud_grid(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> grid;
    grid.setLeafSize(leaf_size,leaf_size,leaf_size);
    grid.setInputCloud(cloud_data[i]);
    grid.filter(*cloud_grid);
    
    //计算点群方差，超出标准差意外标记离群
    pcl::PointCloud<PointT>::Ptr cloud_sor(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_grid);
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*cloud_sor);
    
    *cloud_data[i] = *cloud_sor;
  }
}


void gp::myfilter::get_config()
{
    std::fstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &filter = document["filters"]; 
    leaf_size = filter["leaf_size"].GetFloat();
    nr_k = filter["nr_k"].GetInt();
    stddev_mult = filter["stddev_mult"].GetDouble();
}
