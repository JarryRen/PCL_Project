#pragma once
//Qt
#include <QThread>
#include <QString>
#include <QDebug>
#include <QMetaType>
//pcl
#include <pcl/io/pcd_io.h>
//gp
#include "pcl_project/registration.h"

#include "qt_project/qt_thread_type.h"

class QtICPThread : public QThread
{
    Q_OBJECT
    typedef pcl::PointXYZRGB PointT;
public:
    explicit QtICPThread( QStringList accept_files, QObject *parent = 0 );

signals:
    /**
     * @brief sendPCD copy pcds
     * @param source
     */
    void send(MyCloudType source,MyCloudType  cloud_global);

private:
    QStringList filenames;
    void run();


private slots:
    void stop();
};

