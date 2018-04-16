#include "myfilter.h"

scene::myfilter::myfilter()
{
  get_config();
}

void scene::myfilter::pcd_filter(std::vector< boost::shared_ptr< pcl::PointCloud< scene::PointT > > >& cloud_data)
{
  pcl::PointCloud<PointT>::Ptr cloud;
  for(int i=0; i<cloud_data.size();i++)
  {
    cloud = cloud_data[i];
    pcl::VoxelGrid<PointT> grid;
    grid.setInputCloud(cloud);
    grid.setLeafSize(leaf_size,leaf_size,leaf_size);
    grid.filter(*cloud);

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(nr_k);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*cloud);
  }
}


void scene::myfilter::get_config()
{
    std::fstream fin("project_config.json");
    rapidjson::IStreamWrapper isw(fin);
    
    rapidjson::Document document;
    document.ParseStream(isw);
    
    rapidjson::Value &filter = document["filters"]; 
    leaf_size = filter["leaf_size"].GetDouble();
    nr_k = filter["nr_k"].GetInt();
    stddev_mult = filter["stddev_mult"].GetDouble();
}
