#include "qt_project/qt_viewer.h"
#include "../../build/src/ui_qt_viewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  init_widget();

  connect(ui->action_open_PCD,SIGNAL(triggered(bool)),this,SLOT(open_pcd()));

}

void PCLViewer::init_widget()
{
    cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    viewer->addPointCloud (cloud, "cloud");
    viewer->resetCamera ();
    ui->qvtkWidget->update ();
}

void PCLViewer::open_pcd()
{
    QString filename = QFileDialog::getOpenFileName(
                this,tr("Open PointClud."),".",tr("Open PCD file(*.pcd)"));
    QStringList filenames = QFileDialog::getOpenFileNames(
                this,tr("Open PCD list."),".",tr("Open PCD files(*.pcd)"));

    QStringList::const_iterator itr;
    for(itr = filenames.constBegin();itr!=filenames.constEnd();itr++)
    {
        std::String tmp;
        std::cout << (*itr).toLocal8Bit().constData() <<std::endl;
    }


    if(!filename.isEmpty())
    {
        pcl::io::loadPCDFile(filename.toStdString(),*cloud);
    }
    viewer->updatePointCloud(cloud,"cloud");
    viewer->resetCamera();
    ui->qvtkWidget->update();
}


PCLViewer::~PCLViewer ()
{
  delete ui;
}
