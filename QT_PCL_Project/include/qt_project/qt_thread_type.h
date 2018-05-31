#pragma once
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <QMetaType>

class MyCloudType
{
    typedef pcl::PointXYZRGB PointT;
public:
    MyCloudType() {}
    pcl::PointCloud<PointT> cloud;

};

Q_DECLARE_METATYPE( MyCloudType )
