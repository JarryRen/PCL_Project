#ifndef QT_VIEWER_H
#define QT_VIEWER_H

#include <iostream>
//qt
#include <QMainWindow>
#include <QDebug>
#include <QString>
#include <QFileDialog>
#include <QMetaType>
#include <QVTKWidget.h>
//pcl
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
//gp
#include "qt_project/qt_kinectdata.h"
#include "qt_project/qt_dataprocess.h"
#include "qt_project/qt_icp.h"
#include "qt_project/qt_recon.h"
#include "qt_project/qt_thread_type.h"

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT
    typedef pcl::PointXYZRGB PointT;

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public slots:

    void openPCD();

    /**
     * @brief getKinectData
     */
    void getKinectData();

    /**
    * @brief image_to_pcd
    */
    void imageToPCD();

    /**
    * @brief icp_registration
    * @param[in]
    */
    void icpRegistration();
    /**
     * @brief acceptPCD 接收来自子线程的文件
     * @param source
     * @param cloud_global
     */
    void acceptPCD(MyCloudType source, MyCloudType cloud_global);
    /**
     * @brief saveRegPCD
     */
    void saveRegPCD();

    /**
    * @brief reconstruction
    */
    void reconstruction();
    /**
     * @brief saveMesh
     * @param mesh
     */
    void saveMesh(MyCloudType mesh);

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
  pcl::PointCloud<PointT>::Ptr m_cloud;

  /**
   * @brief initWidgetViewer chushih
   */
  void initWidgetViewer();

private:
  Ui::PCLViewer *ui;
  bool view_not_busy;

};

#endif
