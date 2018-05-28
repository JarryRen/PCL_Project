#include "qt_project/qt_viewer.h"
#include "../../build/src/ui_qt_viewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  initWidget();

  connect(ui->action_open_PCD,SIGNAL(triggered(bool)),this,SLOT(openPCD()));
  connect(ui->pushButton_recon,SIGNAL(triggered(bool)),this,SLOT(reconstruction()));

  //image_to_pcd
  connect(ui->pushButton_imagetopcd,SIGNAL(clicked(bool)), this, SLOT(imageToPCD()));

}

void PCLViewer::initWidget()
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

void PCLViewer::openPCD()
{
    QString filename = QFileDialog::getOpenFileName(
                this,tr("Open PointClud."),".",tr("Open PCD file(*.pcd)"));
    QStringList filenames = QFileDialog::getOpenFileNames(
                this,tr("Open PCD list."),".",tr("Open PCD files(*.pcd)"));

    QStringList::const_iterator itr;
    for(itr = filenames.constBegin();itr!=filenames.constEnd();itr++)
    {
        std::string tmp;
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

void PCLViewer::imageToPCD(){
    QStringList png_filenames = QFileDialog::getOpenFileNames(
                this,tr("选取图像转点云的RGB和深度图像."),".",tr("打开PNG文件(*.png)"));
    QStringList::const_iterator itr;
    for(itr = png_filenames.constBegin();itr!=png_filenames.constEnd();itr++)
    {
        std::string tmp;
        std::cout << (*itr).toLocal8Bit().constData() <<std::endl;
    }

 //   cv
//    gp::DataProcessing dp;
 //   dp.image_to_pcd<pcl::PointXYZRGB>();
}

void PCLViewer::reconstruction()
{
    QString filename = QFileDialog::getOpenFileName(
                this,tr("选取PCD文件."),".",tr("打开文件(*.pcd)"));
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
