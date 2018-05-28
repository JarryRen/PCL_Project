#include "pcl_project/data_processing.h"

gp::DataProcessing::DataProcessing(){

}

void gp::DataProcessing::imageToPCD(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<PointT>::Ptr cloud)
{
    for(int m=0; m<depth.rows; m++)
    {
        for(int n=0; n<depth.cols; n++)
        {
            ushort d = depth.ptr<ushort>(m)[n];
            if( d==0 )
                continue;
            PointT p;
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
}
