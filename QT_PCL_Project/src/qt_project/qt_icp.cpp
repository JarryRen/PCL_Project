#include "qt_project/qt_icp.h"

QtICPThread::QtICPThread(QStringList accept_files, QObject *parent ) :
    QThread( parent )
{
    this->filenames = accept_files;
    qRegisterMetaType< pcl::PointCloud<PointT> >("MyPointType");
    qDebug()<<"配准子线程运行!";
}

void QtICPThread::run()
{
    qDebug()<<"配准开始！ ";
    gp::registration reg;
    Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f icp_trans;
    pcl::PointCloud<PointT>::Ptr global_aligned_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr icp_cloud(new pcl::PointCloud<PointT>);

    pcl::PointCloud<PointT>::Ptr source(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr target(new pcl::PointCloud<PointT>);


    QStringList::const_iterator itr = filenames.begin();
    //第一个target为全局基础
    pcl::io::loadPCDFile( (*itr).toLocal8Bit().constData(), *target );
    itr++;

    MyCloudType send_source;
    MyCloudType send_global;
    //配准
    for( ; itr != filenames.end(); itr++)
    {
        pcl::io::loadPCDFile( (*itr).toLocal8Bit().constData(), *source );

        reg.run(source, target, icp_trans);

        global_transform = global_transform * icp_trans;    //update global transform
        pcl::transformPointCloud(*source, *icp_cloud, global_transform);

        *global_aligned_cloud += *icp_cloud;

        send_source.cloud = *source;
        send_global.cloud = *global_aligned_cloud;
        emit send(send_source,send_global);
       // emit send(*source, *global_aligned_cloud);//send to master thread

        qDebug()<<"配准传送";

        *target = *source;  //更新目标点云
    }

}

void QtICPThread::stop()
{
    qDebug()<<"over";
}
