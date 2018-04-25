#ifndef QT_VIEWER_H
#define QT_VIEWER_H

#include <iostream>
//qt
#include <QMainWindow>
#include <QFileDialog>
//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
//VTK
#include <vtkRenderWindow.h>

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
  void open_pcd();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

  void init_widget();

private:
  Ui::PCLViewer *ui;
};

#endif
