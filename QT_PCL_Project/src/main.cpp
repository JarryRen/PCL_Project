#include <iostream>
//gp
#include "pcl_project/pcl_control.h"

//test
#include <QApplication>
#include <QMainWindow>
#include <qt_project/mainwindow.h>

#include <qt_project/qt_viewer.h>

int main(int argc, char **argv) {
    std::cout << "PCL | Scene" << std::endl;
    
    QApplication a(argc,argv);
 //   MainWindow w;
 //   w.show();
    PCLViewer w;
    w.show ();


    //gp PCL project
    //gp::get_opt_string(argc,argv);
    
    return a.exec();
    //return 0;
}
