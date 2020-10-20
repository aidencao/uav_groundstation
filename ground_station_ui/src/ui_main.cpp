#include <typeinc.h>
#include <ros/ros.h>
#include "mainwindow.h"
#include <QApplication>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    //ROS初始化
    ros::init(argc, argv, "ui_main");
    ROS_INFO("ui_main start");

    qRegisterMetaType<string>("string");
    qRegisterMetaType<geometry_msgs::Vector3>("geometry_msgs::Vector3");
    qRegisterMetaType<sensor_msgs::NavSatFix>("sensor_msgs::NavSatFix");
    qRegisterMetaType<geometry_msgs::PointStamped>("geometry_msgs::PointStamped");
    qRegisterMetaType<geometry_msgs::Vector3Stamped>("geometry_msgs::Vector3Stamped");
    qRegisterMetaType<nav_msgs::Odometry>("nav_msgs::Odometry");


    //QT初始化
    QApplication qa(argc, argv);
    MainWindow w;
    w.setWindowTitle("GroundStation");

    //初始化完毕
    ROS_INFO("init over!");
    ROS_INFO("show MainWindow!");    
    w.show();
    qa.exec();
  
}
