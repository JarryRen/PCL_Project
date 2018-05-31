#include "qt_project/qt_viewer.h"
#include "../../build/src/ui_qt_viewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("PCL viewer");

  //类型注册 用于子线程
  qRegisterMetaType< MyCloudType >("MyCloudType");


  initWidgetViewer();

  connect(ui->action_open_PCD,SIGNAL(triggered(bool)),this,SLOT(openPCD()));
  connect(ui->pushButton_recon,SIGNAL(clicked(bool)),this,SLOT(reconstruction()));

  //image_to_pcd
  connect(ui->pushButton_imageToPCD,SIGNAL(clicked(bool)), this, SLOT(imageToPCD()));
    //icp
  connect(ui->pushButton_icp,SIGNAL(clicked(bool)), this, SLOT(icpRegistration()));


  connect(ui->pushButton,SIGNAL(clicked(bool)), this ,SLOT(test()));

}

void PCLViewer::initWidgetViewer()
{
    m_cloud.reset (new pcl::PointCloud<PointT>);
    m_viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    ui->qvtkWidget->SetRenderWindow (m_viewer->getRenderWindow ());
    m_viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();

    m_viewer->addPointCloud (m_cloud, "cloud");
    m_viewer->resetCamera ();
    ui->qvtkWidget->update ();

}

void PCLViewer::test()
{
    /*
    QtICPThread *t1 = new QtICPThread() ;
    QtICPThread *t2 = new QtICPThread();

    connect(t1, SIGNAL(finished() ),t1,SLOT(stop() ));
    connect(t2, SIGNAL(finished() ), t2,SLOT(stop() ));

    t1->start();
    t2->start();
    */
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
        pcl::io::loadPCDFile(filename.toStdString(),*m_cloud);
    }
    m_viewer->updatePointCloud(m_cloud,"cloud");
    m_viewer->resetCamera();
    ui->qvtkWidget->update();
}

void PCLViewer::imageToPCD(){
    cv::Mat rgb,depth;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    gp::DataProcessing dp;

    QStringList png_filenames = QFileDialog::getOpenFileNames(
                this,tr("选取图像转点云的RGB和深度图像."),".",tr("PNG图像文件(*.png)"));
    QStringList::const_iterator itr;
    QString save_filename = QFileDialog::getSaveFileName(
                this,tr("保存PCD文件."),".",tr("PCD点云文件(*.pcd)"));
    int i=0;
    if( !save_filename.isNull() )
    {
        for(i, itr = png_filenames.constBegin();itr!=png_filenames.constEnd();i++)
        {
            rgb = cv::imread( (*itr).toLocal8Bit().constData() );
            itr++;
            depth = cv::imread( (*itr).toLocal8Bit().constData(), -1 );//onlyread
            itr++;

            dp.imageToPCD(rgb, depth, cloud);

            std::stringstream ss;
            ss<<save_filename.toLocal8Bit().constData()<<"_"<<i<<".pcd";

            pcl::io::savePCDFile( ss.str(),*cloud );
        }
    }
}

void PCLViewer::icpRegistration()
{
    QStringList pcd_filenames = QFileDialog::getOpenFileNames(
                this,tr("选取待配准的点云数据集."),".",tr("PCD点云文件(*.pcd)"));
    if( !pcd_filenames.isEmpty() )
    {
        m_viewer->removePointCloud("source_cloud");
        m_viewer->removePointCloud("global_cloud");
        m_viewer->removePointCloud("cloud");

        m_viewer->addPointCloud(m_cloud,"source_cloud");
        m_viewer->addPointCloud(m_cloud, "global_cloud");
        m_viewer->resetCamera();
        ui->qvtkWidget->update();

        QtICPThread *icp_thread = new QtICPThread(pcd_filenames);
/*直接连接，不够稳定，相当于在子线程直接执行，不安全。
        connect(icpthread,SIGNAL(send(pcl::PointCloud<PointT>,pcl::PointCloud<PointT>)),
                this,SLOT(acceptPCD(pcl::PointCloud<PointT>, pcl::PointCloud<PointT>)),
                Qt::DirectConnection);
    */
        connect(icp_thread,SIGNAL(send(MyCloudType,MyCloudType )),
                this,SLOT(acceptPCD(MyCloudType, MyCloudType ) ) );
        icp_thread->start();

    }

}

void PCLViewer::acceptPCD(MyCloudType source, MyCloudType cloud_global)
{
    qDebug()<<"get PCD in maste thread";
    pcl::PointCloud<PointT>::Ptr src_cloud = source.cloud.makeShared();
    pcl::PointCloud<PointT>::Ptr g_cloud = cloud_global.cloud.makeShared();
    icpUpdateView(src_cloud, g_cloud);
}

void PCLViewer::icpUpdateView(pcl::PointCloud<PointT>::Ptr cloud_src, pcl::PointCloud<PointT>::Ptr cloud_global)
{
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_src(cloud_src, 255, 0, 0 );//red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_global(cloud_global, 0, 255, 0 );//green

    m_viewer->updatePointCloud(cloud_src, color_src, "source_cloud");
    m_viewer->updatePointCloud(cloud_global, color_global, "global_cloud");

    m_viewer->resetCamera();
    ui->qvtkWidget->update();
}



void PCLViewer::reconstruction()
{
    QString filename = QFileDialog::getOpenFileName(
                this,tr("选取PCD文件."),".",tr("打开文件(*.pcd)"));
    if(!filename.isEmpty())
    {
        pcl::io::loadPCDFile(filename.toStdString(),*m_cloud);



    }

    m_viewer->updatePointCloud(m_cloud,"cloud");
    m_viewer->resetCamera();
    ui->qvtkWidget->update();


}



PCLViewer::~PCLViewer ()
{
  delete ui;
}
