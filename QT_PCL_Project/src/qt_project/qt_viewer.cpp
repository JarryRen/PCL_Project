#include "qt_project/qt_viewer.h"
#include "../../build/src/ui_qt_viewer.h"

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  view_not_busy = true;
  this->setWindowTitle ("PCL Viewer");

  //类型注册 用于子线程
  qRegisterMetaType< MyCloudType >("MyCloudType");

  initWidgetViewer();

  connect(ui->action_open_PCD,SIGNAL(triggered(bool)),this,SLOT(openPCD()));
  connect(ui->pushButton_imageToPCD,SIGNAL(clicked(bool)), this, SLOT(imageToPCD()));
  connect(ui->pushButton_icp,SIGNAL(clicked(bool)), this, SLOT(icpRegistration()));
  connect(ui->pushButton_recon,SIGNAL(clicked(bool)),this,SLOT(reconstruction()));
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

void PCLViewer::openPCD()
{
    if(view_not_busy)
    {
        view_not_busy = false;
        QString filename = QFileDialog::getOpenFileName(
                    this,tr("Open PointClud."),".",tr("Open PCD file(*.pcd)"));
        if(!filename.isEmpty())
        {
            ui->textBrowser->append("打开文件:"+filename);
            pcl::io::loadPCDFile(filename.toStdString(),*m_cloud);

            m_viewer->removeAllPointClouds();
            m_viewer->addPointCloud(m_cloud,"cloud");
            m_viewer->resetCamera();
            ui->qvtkWidget->update();
        }
        view_not_busy = true;
    }else
        ui->textBrowser->append("正在忙...");
}

void PCLViewer::imageToPCD(){
    QStringList png_filenames = QFileDialog::getOpenFileNames(
                this,tr("选取图像转点云的RGB和深度图像."),".",tr("PNG图像文件(*.png)"));
    QString save_filename = QFileDialog::getSaveFileName(
                this,tr("保存PCD文件."),".",tr("PCD点云文件(*.pcd)"));
    if(png_filenames.isEmpty() || save_filename.isNull() )
    {
        ui->textBrowser->append("未选取数据或存储目录.");
    }else
    {
        QtDataProcess *dp_thread = new QtDataProcess(png_filenames,save_filename);
        dp_thread->start();
    }
}

void PCLViewer::icpRegistration()
{
    if(view_not_busy)
    {
        view_not_busy = false;
        QStringList pcd_filenames = QFileDialog::getOpenFileNames(
                    this,tr("选取待配准的点云数据集."),".",tr("PCD点云文件(*.pcd)"));
        if( !pcd_filenames.isEmpty() )
        {
            ui->textBrowser->append("开始点云配准.");
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
            connect(icp_thread, SIGNAL(finished() ), this, SLOT( saveRegPCD()));
            icp_thread->start();
        }
    }else
        ui->textBrowser->append("正在忙...");
}

void PCLViewer::acceptPCD(MyCloudType source, MyCloudType cloud_global)
{
    ui->textBrowser->append("接收点云,更新视图...");
    qDebug()<<"get PCD in maste thread";
    pcl::PointCloud<PointT>::Ptr src_cloud = source.cloud.makeShared();
    pcl::PointCloud<PointT>::Ptr g_cloud = cloud_global.cloud.makeShared();

    m_cloud = g_cloud;

    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_src(src_cloud, 255, 0, 0 );//red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_global(g_cloud, 0, 255, 0 );//green

    m_viewer->updatePointCloud(src_cloud, color_src, "source_cloud");
    m_viewer->updatePointCloud(g_cloud, color_global, "global_cloud");

    m_viewer->resetCamera();
    ui->qvtkWidget->update();
}

void PCLViewer::saveRegPCD()
{
    QString save_filename = QFileDialog::getSaveFileName(
                this,tr("保存点云配准PCD文件."),".",tr("PCD点云文件(*.pcd)"));
    if(!save_filename.isNull())
    {
        std::stringstream ss;
        ss<< save_filename.split(".").at(0).toStdString() << ".pcd" ;
        pcl::io::savePCDFile(ss.str(), *m_cloud);
    }else
        ui->textBrowser->append("未保存点云！");

    view_not_busy = true;
    ui->textBrowser->append("当前点云配准流程已完成.");
}

void PCLViewer::reconstruction()
{
    if(view_not_busy)
    {
        view_not_busy=false;
        QString filename = QFileDialog::getOpenFileName(
                    this,tr("选取PCD文件."),".",tr("打开文件(*.pcd)"));
        if(!filename.isEmpty())
        {
            QStringList true_name = filename.split("/");
            ui->textBrowser->append("开始对"+true_name.at(true_name.size()-1)+"模型重建.");
            pcl::io::loadPCDFile(filename.toStdString(),*m_cloud);
            m_viewer->removeAllPointClouds();
            m_viewer->addPointCloud(m_cloud,"cloud");
            m_viewer->resetCamera();
            ui->qvtkWidget->update();

            QtReconThread *recon_thread = new QtReconThread(filename);
            connect(recon_thread, SIGNAL(send(MyCloudType)),
                    this, SLOT(saveMesh(MyCloudType)));
            recon_thread->start();
        }
    }else
        ui->textBrowser->append("正在忙...");
}

void PCLViewer::saveMesh(MyCloudType mesh)
{
    QString save_filename = QFileDialog::getSaveFileName(
                this,tr("保存模型重建VTK文件."),".",tr("VTK点云文件(*.vtk)"));
    if(!save_filename.isNull())
    {
        std::stringstream ss;
        ss<< save_filename.split(".").at(0).toStdString() << ".vtk" ;
        pcl::io::saveVTKFile(ss.str(), mesh.mesh);
    }else
        ui->textBrowser->append("未保存模型！");

    view_not_busy = true;
    ui->textBrowser->append("当前模型重建流程已完成.");
}

void PCLViewer::threadStop()
{
    view_not_busy = true;
    ui->textBrowser->append("当前流程已完成.");
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}
