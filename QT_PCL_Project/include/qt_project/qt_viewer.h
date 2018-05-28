#ifndef QT_VIEWER_H
#define QT_VIEWER_H

#include <iostream>
//qt
#include <QMainWindow>
#include <QFileDialog>
//pcl
#include <pcl/visualization/pcl_visualizer.h>
//VTK
#include <vtkRenderWindow.h>
//gp
#include "pcl_project/loadfile.h"
#include "pcl_project/data_processing.h"
#include "pcl_project/registration.h"
#include "pcl_project/reconstruction.h"

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    typedef pcl::PointXYZRGB PointT;
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

public slots:
  void openPCD();

  /**
   * @brief image_to_pcd
   */
  void imageToPCD();

  /**
   * @brief icp_registration
   * @param[in]
   */
  void icpRegistration();

  void icpUpdateView(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_global);

  /**
   * @brief reconstruction
   */
  void reconstruction();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> m_viewer;
  pcl::PointCloud<PointT>::Ptr m_cloud;

  void initWidgetViewer();

private:
  Ui::PCLViewer *ui;

};

#endif
