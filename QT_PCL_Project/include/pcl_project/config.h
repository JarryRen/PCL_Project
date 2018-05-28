/**
 * 
 * 
 */
#pragma once
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//rapidjson
#include "3rdParty/rapidjson/document.h"
#include "3rdParty/rapidjson/istreamwrapper.h"
#include <fstream>

//Graduation Project
namespace gp{
  typedef pcl::PointXYZRGB PointT;

//声明全局变量，不给初值
  extern std::string g_cfg_path;

}
