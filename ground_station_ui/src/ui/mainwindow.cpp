#include <typeinc.h>
#include <ros/ros.h>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPalette>
#include <QString>
#include <std_msgs/Bool.h>
#include <QDir>
#include <QFileDialog>
#include <qdebug.h>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    pRosTopicThreadHander = new RosTopicSendRecv(this);

    //背景着色
    //QPalette label_palette;
    //label_palette.setColor(QPalette::Background, QColor(102, 164, 146));
    //ui->groupBox_4->setAutoFillBackground(true);
    //ui->groupBox_4->setPalette(label_palette);

    //timerId = this->startTimer(100); //毫秒为单位

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const string loginfo)>(&RosTopicSendRecv::LogInfoSignal),
            this, &MainWindow::on_log_info_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const quint8 gps_health)>(&RosTopicSendRecv::GpsHealthSignal),
            this, &MainWindow::on_gps_health_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const sensor_msgs::NavSatFix)>(&RosTopicSendRecv::GpsInfoSignal),
            this, &MainWindow::on_gps_info_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const geometry_msgs::Vector3Stamped)>(&RosTopicSendRecv::UavVelocitySignal),
            this, &MainWindow::on_uav_velocity_info_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const double)>(&RosTopicSendRecv::HeightAboveTakeoffSignal),
            this, &MainWindow::on_height_above_takeoff_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const geometry_msgs::PointStamped)>(&RosTopicSendRecv::UavLocalPosSignal),
            this, &MainWindow::on_uav_local_pos_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const geometry_msgs::PointStamped)>(&RosTopicSendRecv::LidarNavRelPosSignal),
            this, &MainWindow::on_lidar_nav_rel_pos_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const geometry_msgs::Vector3Stamped)>(&RosTopicSendRecv::LidarNavComputedVelSignal),
            this, &MainWindow::on_lidar_nav_computed_vel_slot);

    connect(pRosTopicThreadHander,
            static_cast<void (RosTopicSendRecv::*)(const nav_msgs::Odometry)>(&RosTopicSendRecv::LidarNavOdomSignal),
            this, &MainWindow::on_lidar_nav_odom_slot);

    pRosTopicThreadHander->start();

    //控制输入值
    QRegExp doubleRegx("[0-9.0-9]*");                      //非负小数
    QRegExp posRegx("^[0-9]*$");                           //非负整数
    QRegExp intRegx("^(0|[1-9][0-9]*|-[1-9][0-9]*)$");     //整数
    QRegExp alldoubleRegx("^(-?[0-9])|(-?\\d+)(\.\\d+)$"); //小数

    ui->octoResolution->setValidator(new QRegExpValidator(doubleRegx, ui->octoResolution));
    ui->max_range->setValidator(new QRegExpValidator(posRegx, ui->max_range));
    ui->max_z->setValidator(new QRegExpValidator(intRegx, ui->max_z));
    ui->min_z->setValidator(new QRegExpValidator(intRegx, ui->min_z));

    ui->step_range->setValidator(new QRegExpValidator(doubleRegx, ui->step_range));
    ui->bound_xy->setValidator(new QRegExpValidator(posRegx, ui->bound_xy));
    ui->bound_highz->setValidator(new QRegExpValidator(intRegx, ui->bound_highz));
    ui->bound_lowz->setValidator(new QRegExpValidator(intRegx, ui->bound_lowz));
    ui->uavl->setValidator(new QRegExpValidator(doubleRegx, ui->uavl));
    ui->uavw->setValidator(new QRegExpValidator(doubleRegx, ui->uavw));
    ui->uavh->setValidator(new QRegExpValidator(doubleRegx, ui->uavh));
    ui->box_size->setValidator(new QRegExpValidator(doubleRegx, ui->box_size));
    ui->uav_hight->setValidator(new QRegExpValidator(doubleRegx, ui->uav_hight));
    ui->point_x->setValidator(new QRegExpValidator(alldoubleRegx, ui->point_x));
    ui->point_y->setValidator(new QRegExpValidator(alldoubleRegx, ui->point_y));
    ui->point_z->setValidator(new QRegExpValidator(alldoubleRegx, ui->point_z));
    //文本颜色
    ui->loadMap_warn->setStyleSheet("color:red");
    ui->setPoint_warn->setStyleSheet("color:red");
}

MainWindow::~MainWindow()
{
    delete ui;
}

//void MainWindow::timerEvent(QTimerEvent *e){}

////////////////////////////////////////////////////////////////////////////
//  button click
////////////////////////////////////////////////////////////////////////////

void MainWindow::on_takeoffBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_takeoff_cmd()");
    pRosTopicThreadHander->pub_takeoff_cmd();
}

void MainWindow::on_landBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_land_cmd()");
    pRosTopicThreadHander->pub_land_cmd();
}

void MainWindow::on_haltMFBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_haltManifold_cmd()");
    pRosTopicThreadHander->pub_haltManifold_cmd();
}

void MainWindow::on_goHomeBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_gohome_cmd()");
    pRosTopicThreadHander->pub_gohome_cmd();
}

void MainWindow::on_rstHeightBtn_clicked()
{
    double height = ui->rstHeightLedt->text().toDouble();
    ROS_INFO("pRosTopicHander->pub_resetHeight_cmd()");
    pRosTopicThreadHander->pub_resetHeight_cmd(height);
}

void MainWindow::on_lidarNavActivateBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_activateLidarNav_cmd()");
    pRosTopicThreadHander->pub_activateLidarNav_cmd();
}

void MainWindow::on_lidarNavPauseBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_pauseLidarNav_cmd()");
    pRosTopicThreadHander->pub_pauseLidarNav_cmd();
}

void MainWindow::on_lidarNavResumeBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_resumeLidarNav_cmd()");
    pRosTopicThreadHander->pub_resumeLidarNav_cmd();
}

void MainWindow::on_lidarNavCancelBtn_clicked()
{
    ROS_INFO("pRosTopicHander->pub_cancelLidarNav_cmd()");
    pRosTopicThreadHander->pub_cancelLidarNav_cmd();
}

void MainWindow::on_lidarNavSetMaxSpeedBtn_clicked()
{
    double vel = ui->lidarNavSetMaxSpeedLedit->text().toDouble();
    ROS_INFO("pRosTopicHander->pub_resetMaxVelocity_cmd()");
    pRosTopicThreadHander->pub_resetMaxVelocity_cmd(vel);
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    UINT8 mode = index;
    cout << "mode: " << (int)mode << endl;
    ROS_INFO("pRosTopicHander->pub_change_flight_mode_cmd()");
    pRosTopicThreadHander->pub_change_flight_mode_cmd(mode);
}

////////////////////////////////////////////////////////////////////////////
//  slot
////////////////////////////////////////////////////////////////////////////

void MainWindow::on_log_info_slot(const string loginfo)
{
    ROS_INFO("MainWindow::on_log_info_slot");
    ui->logInfoTxt->append(QString(loginfo.c_str()));
}

void MainWindow::on_gps_info_slot(const sensor_msgs::NavSatFix gpsinfo)
{
    ROS_INFO("MainWindow::on_gps_info_slot");
    QString gspInfo = QString("Latitude: %1\nLongitude: %2\nAltitude: %3")
                          .arg(gpsinfo.latitude)
                          .arg(gpsinfo.longitude)
                          .arg(gpsinfo.altitude);
    ui->gpsInfoLbl->setText(gspInfo);
}

void MainWindow::on_gps_health_slot(const quint8 gps_health)
{
    ROS_INFO("MainWindow::on_gps_health_slot");
    QString gspHealthStr = QString("GPS Health: %1").arg(gps_health);
    ui->gpsHealthLbl->setText(gspHealthStr);
}

void MainWindow::on_uav_velocity_info_slot(const geometry_msgs::Vector3Stamped vel)
{
    ROS_INFO("MainWindow::on_uav_velocity_info_slot");
    QString VelStr = QString("Velocity X:%1\nVelocity Y:%2\nVelocity Z:%3")
                         .arg(vel.vector.x)
                         .arg(vel.vector.y)
                         .arg(vel.vector.z);
    ui->uavVelLbl->setText(VelStr);
}

void MainWindow::on_height_above_takeoff_slot(const double height)
{
    ROS_INFO("MainWindow::on_height_above_takeoff_slot");
    QString heightStr = QString("Height Above Takeoff: %1").arg(height);
    ui->heightAboveTakeoffLbl->setText(heightStr);
}

void MainWindow::on_uav_local_pos_slot(const geometry_msgs::PointStamped local_pos)
{
    ROS_INFO("MainWindow::on_uav_local_pos_slot");
    QString posStr = QString("Local Position X:%1\nLocal Position Y:%2\nLocal Position Z:%3")
                         .arg(local_pos.point.x)
                         .arg(local_pos.point.y)
                         .arg(local_pos.point.z);
    ui->localPosLbl->setText(posStr);
}

void MainWindow::on_lidar_nav_rel_pos_slot(const geometry_msgs::PointStamped rel_pos)
{
    double distance = sqrt(pow(rel_pos.point.x, 2) + pow(rel_pos.point.y, 2));
    ROS_INFO("MainWindow::on_lidar_nav_rel_pos_slot");
    QString posStr = QString("Computed Rel PosX:%1\nComputed Rel PosY:%2\nComputed Rel PosZ:%3\nComputed Rel Distance:%4")
                         .arg(rel_pos.point.x)
                         .arg(rel_pos.point.y)
                         .arg(rel_pos.point.z)
                         .arg(distance);
    ui->computedRefPosLbl->setText(posStr);
}

void MainWindow::on_lidar_nav_computed_vel_slot(const geometry_msgs::Vector3Stamped vel)
{
    ROS_INFO("MainWindow::on_lidar_nav_computed_vel_slot");
    QString velStr = QString("Computed Vel X:%1\nComputed Vel Y:%2\nComputed Vel Z:%3")
                         .arg(vel.vector.x)
                         .arg(vel.vector.y)
                         .arg(vel.vector.z);
    ui->computedVelLbl->setText(velStr);
}

void MainWindow::on_lidar_nav_odom_slot(const nav_msgs::Odometry odom)
{
    ROS_INFO("MainWindow::on_lidar_nav_odom_slot");
    QString odomStr = QString("Odom Pos X:%1\nOdom Pos Y:%2\nOdom Pos Z:%3\n"
                              "Odom Quat W:%4\nOdom Quat X:%5\nOdom Quat Y:%6\nOdom Quat Z:%7")
                          .arg(odom.pose.pose.position.x)
                          .arg(odom.pose.pose.position.y)
                          .arg(odom.pose.pose.position.z)
                          .arg(odom.pose.pose.orientation.w)
                          .arg(odom.pose.pose.orientation.x)
                          .arg(odom.pose.pose.orientation.y)
                          .arg(odom.pose.pose.orientation.z);
    ui->odomPosLbl->setText(odomStr);
}

void MainWindow::on_select_file_clicked()
{
    ROS_INFO("MainWindow::on_select_file_clicked");
    QString directory =
        QDir::toNativeSeparators(QFileDialog::getOpenFileName(this, tr("Load Map"), QString::fromStdString(pRosTopicThreadHander->map_dir), tr("PCD (*.pcd)")));

    if (!directory.isEmpty())
    {
        ui->mapPath->setText(directory);
    }
    else
    {
        ui->mapPath->setText("");
    }
}

void MainWindow::on_loadMap_clicked(bool checked)
{
    if (checked)
    {
        ROS_INFO("MainWindow::on_loadMap_clicked");
        ui->loadMap_warn->setText("");

        QString path = ui->mapPath->text();
        if (path.isNull() || path.isEmpty())
        {
            ui->loadMap_warn->setText("路径不能为空");
            return;
        }

        QString end = " &";

        //拼接octomap_server终端命令
        QString command = "roslaunch publish_pointcloud octomap_server.launch";
        //各个参数
        QString resolution = ui->octoResolution->text();
        if (!resolution.isNull() && !resolution.isEmpty())
        {
            command = command + " resolution:=" + resolution;
        }
        QString max_range = ui->max_range->text();
        if (!max_range.isNull() && !max_range.isEmpty())
        {
            command = command + " max_range:=" + max_range;
        }
        QString max_z = ui->max_z->text();
        if (!max_z.isNull() && !max_z.isEmpty())
        {
            command = command + " max_z:=" + max_z;
        }
        QString min_z = ui->min_z->text();
        if (!min_z.isNull() && !min_z.isEmpty())
        {
            command = command + " min_z:=" + min_z;
        }
        // octo_server_command = octo_server_command + end;
        // system(octo_server_command.toLatin1());

        //拼接planner终端命令
        // QString path_planner_command = "roslaunch path_planner path_planner_qt.launch";
        // if (!resolution.isNull() && !resolution.isEmpty())
        // {
        //     path_planner_command = path_planner_command + " resolution:=" + resolution;
        // }
        QString step_range = ui->step_range->text();
        if (!step_range.isNull() && !step_range.isEmpty())
        {
            command = command + " step_range:=" + step_range;
        }
        QString bound_xy = ui->bound_xy->text();
        if (!bound_xy.isNull() && !bound_xy.isEmpty())
        {
            command = command + " bound_xy:=" + bound_xy;
        }
        QString bound_highz = ui->bound_highz->text();
        if (!bound_highz.isNull() && !bound_highz.isEmpty())
        {
            command = command + " bound_highz:=" + bound_highz;
        }
        QString bound_lowz = ui->bound_lowz->text();
        if (!bound_lowz.isNull() && !bound_lowz.isEmpty())
        {
            command = command + " bound_lowz:=" + bound_lowz;
        }
        QString uavl = ui->uavl->text();
        if (!uavl.isNull() && !uavl.isEmpty())
        {
            command = command + " uavl:=" + uavl;
        }
        QString uavw = ui->uavw->text();
        if (!uavw.isNull() && !uavw.isEmpty())
        {
            command = command + " uavw:=" + uavw;
        }
        QString uavh = ui->uavh->text();
        if (!uavh.isNull() && !uavh.isEmpty())
        {
            command = command + " uavh:=" + uavh;
        }
        // path_planner_command = path_planner_command + end;
        // system(path_planner_command.toLatin1());

        //拼接gps_mapping终端命令
        // QString gps_mapping_command = "roslaunch gps_mapping gps_mapping_qt.launch";
        //各个参数
        QString box_size = ui->box_size->text();
        if (!box_size.isNull() && !box_size.isEmpty())
        {
            command = command + " box_size:=" + box_size;
        }

        // gps_mapping_command = gps_mapping_command + end;
        // system(gps_mapping_command.toLatin1());

        //拼接地图发送终端命令
        // QString command = "roslaunch publish_pointcloud publish_map.launch";
        command = command + " path:=" + path;
        command = command + end;
        system(command.toLatin1());
    }
    else
    {
        ROS_INFO("MainWindow::on_loadMap_released");
        QString command = "rosnode kill /path_planner_node;rosnode kill /octomap_server;rosnode kill /gps_mapping_node";
        system(command.toLatin1());
        // command = "gnome-terminal -x bash -c 'rqt_image_view;'";
        command = "rosnode kill $(rosnode list | grep rviz)";
        system(command.toLatin1());
        command = "killall rviz";
        system(command.toLatin1());
    }
}

void MainWindow::on_record_start_clicked()
{
    ROS_INFO("pRosTopicThreadHander->on_record_start_clicked()");
    pRosTopicThreadHander->pub_record_start_cmd();
}

void MainWindow::on_record_end_clicked()
{
    ROS_INFO("pRosTopicThreadHander->on_record_end_clicked()");
    pRosTopicThreadHander->pub_record_end_cmd();
}

// void MainWindow::on_planner_start_clicked()
// {
//     ROS_INFO("MainWindow::on_planner_start_clicked");
//     //拼接终端命令
//     QString command = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch path_planner path_planner_qt.launch";
//     QString end = ";'";
//     QString resolution = ui->octoResolution->text();
//     if (!resolution.isNull() && !resolution.isEmpty())
//     {
//         command = command + " resolution:=" + resolution;
//     }
//     QString step_range = ui->step_range->text();
//     if (!step_range.isNull() && !step_range.isEmpty())
//     {
//         command = command + " step_range:=" + step_range;
//     }
//     QString bound_xy = ui->bound_xy->text();
//     if (!bound_xy.isNull() && !bound_xy.isEmpty())
//     {
//         command = command + " bound_xy:=" + bound_xy;
//     }
//     QString bound_highz = ui->bound_highz->text();
//     if (!bound_highz.isNull() && !bound_highz.isEmpty())
//     {
//         command = command + " bound_highz:=" + bound_highz;
//     }
//     QString bound_lowz = ui->bound_lowz->text();
//     if (!bound_lowz.isNull() && !bound_lowz.isEmpty())
//     {
//         command = command + " bound_lowz:=" + bound_lowz;
//     }
//     QString uavl = ui->uavl->text();
//     if (!uavl.isNull() && !uavl.isEmpty())
//     {
//         command = command + " uavl:=" + uavl;
//     }
//     QString uavw = ui->uavw->text();
//     if (!uavw.isNull() && !uavw.isEmpty())
//     {
//         command = command + " uavw:=" + uavw;
//     }
//     QString uavh = ui->uavh->text();
//     if (!uavh.isNull() && !uavh.isEmpty())
//     {
//         command = command + " uavh:=" + uavh;
//     }
//     command = command + end;
//     system(command.toLatin1());
// }

void MainWindow::on_set_height_clicked()
{
    ROS_INFO("MainWindow::on_set_height_clicked");
    double height = ui->uav_hight->text().toDouble();
    pRosTopicThreadHander->pub_set_height_by_move_cmd(height);
}

void MainWindow::on_tags_start_clicked(bool checked)
{
    if (checked)
    {
        ROS_INFO("MainWindow::on_tags_start_clicked");
        //拼接终端命令
        // QString command = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch uav_landing April_tag_camera.launch;'";
        QString command = "roslaunch uav_landing April_tag_camera.launch &";
        system(command.toLatin1());
        // command = "gnome-terminal -x bash -c 'rqt_image_view;'";
        command = "rqt_image_view &";
        system(command.toLatin1());
        // command = "gnome-terminal -x bash -c 'source ~/.bashrc;rosrun uav_landing listener_tags_together.py;'";
        command = "rosrun uav_landing listener_tags_together.py &";
        system(command.toLatin1());
    }
    else
    {
        ROS_INFO("MainWindow::on_tags_start_released");
        QString command = "rosnode kill /usb_cam;rosnode kill /apriltag_ros_continuous_node;rosnode kill /listener_node";
        system(command.toLatin1());
        // command = "gnome-terminal -x bash -c 'rqt_image_view;'";
        command = "rosnode kill $(rosnode list | grep rqt_gui_cpp_node)";
        system(command.toLatin1());
        command = "killall rqt_image_view";
        system(command.toLatin1());
    }
}

void MainWindow::on_landing_start_clicked()
{
    ROS_INFO("pRosTopicThreadHander->pub_vision_land_start_cmd()");
    pRosTopicThreadHander->pub_vision_land_start_cmd();
}

// void MainWindow::on_rviz_start_clicked()
// {
//     ROS_INFO("MainWindow::on_rviz_start_clicked()");
//     //拼接终端命令
//     QString command = "gnome-terminal -x bash -c 'source ~/.bashrc;roslaunch gps_mapping gps_mapping_qt.launch";
//     QString end = ";'";
//     //各个参数
//     QString box_size = ui->box_size->text();
//     if (!box_size.isNull() && !box_size.isEmpty())
//     {
//         command = command + " box_size:=" + box_size;
//     }

//     command = command + end;
//     system(command.toLatin1());
// }

void MainWindow::on_set_point_clicked()
{
    ROS_INFO("pRosTopicThreadHander->on_set_point_clicked()");
    ui->setPoint_warn->setText("");
    QString x = ui->point_x->text();
    if (x.isNull() || x.isEmpty())
    {
        ui->setPoint_warn->setText("坐标不全");
        return;
    }
    QString y = ui->point_y->text();
    if (y.isNull() || y.isEmpty())
    {
        ui->setPoint_warn->setText("坐标不全");
        return;
    }
    QString z = ui->point_z->text();
    if (z.isNull() || z.isEmpty())
    {
        ui->setPoint_warn->setText("坐标不全");
        return;
    }

    pRosTopicThreadHander->pub_set_point_cmd(x.toDouble(),y.toDouble(),z.toDouble());
}