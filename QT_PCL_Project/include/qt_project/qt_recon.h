#pragma once
//Qt
#include <QThread>
#include <QString>
#include <QDebug>
#include <QMetaType>
//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//vtk
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>

//gp
#include "pcl_project/reconstruction.h"
#include "qt_project/qt_thread_type.h"

class QtReconThread : public QThread
{
    Q_OBJECT
    typedef pcl::PointXYZRGB PointT;
public:
    explicit QtReconThread( QString accept_file, QObject *parent = 0 );

public slots:
    void stop();

private:
    QString filename;
    void run();
};
