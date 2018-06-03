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

void gp::DataProcessing::dataProcess(pcl::PointCloud<PointT>::Ptr cloud, pcl::PointCloud<PointT>::Ptr cloud_processed)
{
    //去除NAN点 not a number
    std::vector<int> indieces;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indieces);

    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::PointCloud<PointT>::Ptr cloud_pass_flitered(new pcl::PointCloud<PointT>);
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0,1);//自己环境的配置
    pass.filter(*cloud_pass_flitered);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(cloud_pass_flitered);
    seg.segment(*inliers,*coeff);

    pcl::ExtractIndices<PointT> extract;
    pcl::PointCloud<PointT>::Ptr cloud_no_plane(new pcl::PointCloud<PointT>);
    extract.setInputCloud(cloud_pass_flitered);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_no_plane);

    //离群点
    pcl::PointCloud<PointT>::Ptr cloud_sor_filtered(new pcl::PointCloud<PointT>);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_no_plane);
    sor.setMeanK(3000);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_sor_filtered);

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud_sor_filtered);
    ror.setRadiusSearch(0.1);
    ror.setMinNeighborsInRadius(200);
    ror.filter(*cloud_processed);
}
