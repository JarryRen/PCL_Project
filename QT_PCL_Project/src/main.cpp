#include <iostream>
//qt
#include <QApplication>
#include <QMainWindow>
#include <qt_project/mainwindow.h>
//gp
#include "pcl_project/pcl_control.h"

#include <qt_project/qt_viewer.h>

int main(int argc, char **argv) {
    std::cout << "PCL | Object" << std::endl;

    if(argc == 1)
    {
        //qt pcl project
        QApplication a(argc,argv);
        PCLViewer w;
        w.show ();
        return a.exec();
    }
    else{
         //gp PCL project
        gp::get_opt_string(argc,argv);
        return 0;
    }

    return 0;
}
