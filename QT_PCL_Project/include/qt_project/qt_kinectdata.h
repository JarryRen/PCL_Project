#pragma once
//Qt
#include <QThread>
#include <QString>
#include <QDebug>
#include <QMetaType>
//pcl
#include <pcl/io/pcd_io.h>
//openni opencv
#include <XnCppWrapper.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//gp
#include "qt_project/qt_thread_type.h"

class QtKinect : public QThread
{
    Q_OBJECT
    typedef pcl::PointXYZRGB PointT;
public:
    explicit QtKinect( QString path, QObject *parent = 0 );
    void run();
private:
    QString save_path;
};

