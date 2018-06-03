#pragma once
//Qt
#include <QThread>
#include <QString>
#include <QDebug>
#include <QMetaType>
//pcl
#include <pcl/io/pcd_io.h>
//gp
#include "pcl_project/data_processing.h"

#include "qt_project/qt_thread_type.h"

class QtDataProcess : public QThread
{
    Q_OBJECT
    typedef pcl::PointXYZRGB PointT;
public:
    explicit QtDataProcess( QStringList list, QString path, QObject *parent = 0 );
    void run();
private:
    QStringList data_set;
    QString save_path;
};
