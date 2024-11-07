#include "vizlib_test.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    // ROS 초기화
    ros::init(argc, argv, "vizlib_test_node");

    QApplication a(argc, argv);
    VizlibTest w(argc, argv);
    w.show();

    return a.exec();
}
