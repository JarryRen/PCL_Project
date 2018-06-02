#pragma once
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <QMetaType>

class MyCloudType
{
    typedef pcl::PointXYZRGB PointT;
public:
    MyCloudType() {}
    pcl::PointCloud<PointT> cloud;
    pcl::PolygonMesh mesh;
};

Q_DECLARE_METATYPE( MyCloudType )
