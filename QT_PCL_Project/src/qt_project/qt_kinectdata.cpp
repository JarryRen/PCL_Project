#include "qt_project/qt_kinectdata.h"

QtKinect::QtKinect( QString path, QObject *parent):
    QThread( parent )
{
    this->save_path = path.split(".").at(0);
}

void QtKinect::run()
{
    //https://kheresy.wordpress.com/2012/07/25/openni-and-opencv/
    //初始化OpenNI环境
    xn::Context xn_context;
    xn_context.Init();
    xn::ImageGenerator img_generator;
    xn::DepthGenerator depth_generator;
    img_generator.Create(xn_context);
    depth_generator.Create(xn_context);
    //设置获取大小
    XnMapOutputMode map_mode;
    map_mode.nXRes = 640;
    map_mode.nYRes = 480;
    map_mode.nFPS = 30;
    img_generator.SetMapOutputMode( map_mode );
    depth_generator.SetMapOutputMode( map_mode );
    //设置视点
    depth_generator.GetAlternativeViewPointCap().SetViewPoint(img_generator);

    //元数据
    xn::ImageMetaData rgb_MD;
    xn::DepthMetaData depth_MD;

    cv::namedWindow("RGB Image",1);
    cv::namedWindow("Depth Image",1);

    xn_context.StartGeneratingAll();
    xn_context.WaitAndUpdateAll();

    int i = 0;
    char key = 0;//null
    while( key != 'q' )//esc
    {
        xn_context.WaitAndUpdateAll();
        img_generator.GetMetaData(rgb_MD);
        depth_generator.GetMetaData(depth_MD);

        cv::Mat rgb_img(rgb_MD.FullYRes(), rgb_MD.FullXRes(), CV_8UC3, (void*)rgb_MD.Data());
        cv::Mat bgr_img_show;
        cv::cvtColor(rgb_img, bgr_img_show, CV_RGB2BGR);
        cv::imshow("RGB Image",bgr_img_show);

        cv::Mat depth_img(depth_MD.FullYRes(), depth_MD.FullXRes(), CV_16UC1, (void*)depth_MD.Data());
        cv::Mat depth_img_show;
        depth_img.convertTo(depth_img_show, CV_8U,255.0/7000);
        cv::imshow("Depth Image",depth_img_show);

        key=cvWaitKey(20);

        switch ( key ) {
        case 's':
        {
            std::stringstream ss_rgb;
            ss_rgb<<save_path.toStdString()<<"_"<<i<<".png";
            cv::imwrite(ss_rgb.str(),bgr_img_show);

            std::stringstream ss_depth;
            ss_depth<<save_path.toStdString()<<"_"<<i++<<"_depth.png";
            cv::imwrite(ss_depth.str(),depth_img);
            break;
        }
        default:
            break;
        }
    }
    cvDestroyWindow("RGB Image");
    cvDestroyWindow("Depth Image");
    xn_context.StopGeneratingAll();
    xn_context.Release();
}
