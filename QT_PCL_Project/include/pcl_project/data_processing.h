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
    public:
        DataProcessing();

        /**
         * @brief imageToPCD
         * @param rgb
         * @param depth
         *
         */
        template<typename PointX>
        void imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointX>& cloud)
        {
            for(int m=0; m<depth.rows; m++)
            {
                for(int n=0; n<depth.cols; n++)
                {
                    ushort d = depth.ptr<ushort>(m)[n];
                    if( d==0 )
                        continue;
                    PointX p;
                    p.z=double(d) / CAMERA_FACTOR;
                    p.x=(n-CAMERA_CX)*p.z/CAMERA_FX;
                    p.y=(m-CAMERA_CY)*p.z/CAMERA_FY;

                    p.b = rgb.ptr<uchar>(m)[n*3];
                    p.g = rgb.ptr<uchar>(m)[n*3+1];
                    p.r = rgb.ptr<uchar>(m)[n*3+2];

                    cloud->points.push_back(p);
                }
            }
            cloud->height = 1;
            cloud->width = cloud->points.size();
            cloud->is_dense =false;
        };

    private:
        //KinectV1相机内参
        const double CAMERA_FACTOR = 1000;
        const double CAMERA_CX = 325.5;
        const double CAMERA_CY = 253.5;
        const double CAMERA_FX = 518.0;
        const double CAMERA_FY = 519.0;
    };
}


