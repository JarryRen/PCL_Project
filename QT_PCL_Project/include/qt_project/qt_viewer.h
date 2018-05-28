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
#include "pcl_project/reconstruction.h"
#include "pcl_project/data_processing.h"

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
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
 // void icp_registration();
  /**
   * @brief reconstruction
   */
  void reconstruction();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  void initWidget();

private:
  Ui::PCLViewer *ui;
};

#endif
