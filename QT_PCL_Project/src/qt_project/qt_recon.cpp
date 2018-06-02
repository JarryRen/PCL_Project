#include "qt_project/qt_recon.h"

QtReconThread::QtReconThread( QString accept_file, QObject *parent ) :
    QThread( parent )
{
    this->filename = accept_file;
    qDebug()<<"模型重建。";
}

void QtReconThread::run()
{
    gp::reconsruction recon;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PolygonMesh mesh;
    pcl::io::loadPCDFile(filename.toStdString(),*cloud);
    recon.run(cloud,mesh);

    //显示
    vtkSmartPointer<vtkPolyData> vtk_mesh;
    pcl::VTKUtils::mesh2vtk(mesh, vtk_mesh);
    //映射和演员actor
    vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    mapper->SetInputData(vtk_mesh);
    vtkActor *actor = vtkActor::New();
    actor->SetMapper(mapper);
    //渲染 渲染窗口 交互器 交互方式
    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    vtkRenderWindow *render_window = vtkRenderWindow::New();
    render_window->AddRenderer(renderer);
    render_window->SetSize(500,500);
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(render_window);
    vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
    iren->SetInteractorStyle(style);

    iren->Initialize();
    iren->Start();

    //必须先关闭交互器 https://www.vtk.org/Wiki/VTK/Examples/Cxx/Visualization/CloseWindow
    iren->GetRenderWindow()->Finalize();
    iren->TerminateApp();

    //关闭窗口及其他文件
    style->Delete();
    iren->Delete();
    render_window->Delete();
    renderer->Delete();
    actor->Delete();
    mapper->Delete();
}

void QtReconThread::stop()
{
    this->quit();
    qDebug()<< "over";
}
