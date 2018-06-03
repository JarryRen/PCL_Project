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
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
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
         * @brief imageToPCD 图像转点云
         * @param rgb
         * @param depth
         *
         */
        void imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointT>::Ptr cloud);

        /**
         * @brief DataProcess 进行分割处理
         * @param cloud
         * @param cloud_processed
         */
        void dataProcess(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_processed);
    };
}


