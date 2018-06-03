#include "qt_project/qt_dataprocess.h"

QtDataProcess::QtDataProcess( QStringList list, QString path, QObject *parent ) :
    QThread( parent )
{
    this->data_set = list;
    this->save_path = path;
    save_path =save_path.split(".").at(0);//不需要后缀
}

void QtDataProcess::run()
{
    cv::Mat rgb,depth;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_processed(new pcl::PointCloud<PointT>);
    gp::DataProcessing dp;

    QStringList::const_iterator itr;
    int i=0;
    for(i, itr = data_set.constBegin();itr!=data_set.constEnd();i++)
    {
        rgb = cv::imread( (*itr).toLocal8Bit().constData() );
        itr++;
        depth = cv::imread( (*itr).toLocal8Bit().constData(), -1 );//onlyread
        itr++;

        dp.imageToPCD(rgb, depth, cloud);
        dp.dataProcess(cloud, cloud_processed);

        std::stringstream ss;
        ss<<save_path.toStdString()<<"_"<<i<<".pcd";
        pcl::io::savePCDFile( ss.str(),*cloud_processed);
    }
}
