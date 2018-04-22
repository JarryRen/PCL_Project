#include <iostream>
//gp
#include "pcl_project/pcl_control.h"

//test
#include <QApplication>
#include <QMainWindow>
#include <qt_project/mainwindow.h>

int main(int argc, char **argv) {
    std::cout << "PCL | Scene" << std::endl;
    
    QApplication a(argc,argv);
    MainWindow w;
    w.show();



    //gp PCL project
    //gp::get_opt_string(argc,argv);
    
    return a.exec();
}
