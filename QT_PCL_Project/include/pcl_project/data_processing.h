#pragma once
#include <iostream>
#include <string>
#include <fstream>
//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//rapidjson
#include "3rdParty/rapidjson/document.h"
#include "3rdParty/rapidjson/istreamwrapper.h"
//gp

namespace gp{
/**
     * @brief The DataProcessing classs
     *
     */
    class DataProcessing
    {
        typedef pcl::PointXYZRGB PointT;
    private:
        //KinectV1相机内参
        const double CAMERA_FACTOR = 1000;
        const double CAMERA_CX = 325.5;
        const double CAMERA_CY = 253.5;
        const double CAMERA_FX = 518.0;
        const double CAMERA_FY = 519.0;

    public:
        DataProcessing();

        /**
         * @brief imageToPCD
         * @param rgb
         * @param depth
         *
         */
        void imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointT>::Ptr cloud);

    };
}


