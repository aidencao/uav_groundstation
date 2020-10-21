#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "typeinc.h"
#include "ros_topic_send_recv.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    //void timerEvent(QTimerEvent *e);

private slots:
    //button
    void on_takeoffBtn_clicked();
    void on_landBtn_clicked();
    void on_haltMFBtn_clicked();
    void on_goHomeBtn_clicked();
    void on_rstHeightBtn_clicked();
    void on_lidarNavActivateBtn_clicked();
    void on_lidarNavPauseBtn_clicked();
    void on_lidarNavResumeBtn_clicked();
    void on_lidarNavCancelBtn_clicked();
    void on_lidarNavSetMaxSpeedBtn_clicked();
    //uav info
    void on_log_info_slot(const string loginfo);
    void on_gps_info_slot(const sensor_msgs::NavSatFix gpsinfo);
    void on_gps_health_slot(const quint8 gps_health);
    void on_uav_velocity_info_slot(const geometry_msgs::Vector3Stamped vel);
    void on_height_above_takeoff_slot(const double height);
    void on_uav_local_pos_slot(const geometry_msgs::PointStamped local_pos);
    void on_lidar_nav_rel_pos_slot(const geometry_msgs::PointStamped rel_pos);
    void on_lidar_nav_computed_vel_slot(const geometry_msgs::Vector3Stamped vel);
    void on_lidar_nav_odom_slot(const nav_msgs::Odometry odom);
    void on_comboBox_currentIndexChanged(int index);

    //loadmap
    void on_select_file_clicked();

    void on_loadMap_clicked();

    void on_planner_start_clicked();
    void on_tags_start_clicked(bool checked);
    void on_landing_start_clicked();
    void on_rviz_start_clicked();

private:
    Ui::MainWindow *ui;
    //int timerId;//定时器id
    RosTopicSendRecv *pRosTopicThreadHander;
};

#endif // MAINWINDOW_H
