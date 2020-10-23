#include "ros_topic_send_recv.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <functional>

RosTopicSendRecv::RosTopicSendRecv(QObject *parent)
    : QThread(parent)
{
    nh.param("ui_main/dir", map_dir, std::string("/home/cyr/nav_ws/src/publish_pointcloud/data"));

    //ros publisher,要放到构造函数，如果放到run，可能线程还没起，但可能因为按键被调用了
    takeoff_pub = nh.advertise<std_msgs::Bool>("/uav_take_off", 1);
    land_pub = nh.advertise<std_msgs::Bool>("/uav_land", 1);
    gohome_pub = nh.advertise<std_msgs::Bool>("/uav_gohome", 1);
    halt_manifold_pub = nh.advertise<std_msgs::Bool>("/uav_halt_manifold", 1);
    reset_height_pub = nh.advertise<std_msgs::Float64>("/set_uav_height", 1);
    activate_lidar_nav_pub = nh.advertise<std_msgs::Bool>("/lidar_nav_activate", 1);
    pause_lidar_nav_pub = nh.advertise<std_msgs::Bool>("/lidar_nav_pause", 1);
    resume_lidar_nav_pub = nh.advertise<std_msgs::Bool>("/lidar_nav_resume", 1);
    cancel_lidar_nav_pub = nh.advertise<std_msgs::Bool>("/lidar_nav_cancel", 1);
    reset_lidar_nav_max_vel_pub = nh.advertise<std_msgs::Float64>("/lidar_nav_set_max_vel", 1);
    change_lidar_nav_flight_mode_pub = nh.advertise<std_msgs::UInt8>("/lidar_nav_change_mode", 1);
    vision_land_start_pub = nh.advertise<std_msgs::Bool>("/qt_landing_start", 1);
    record_map_start_pub = nh.advertise<std_msgs::Bool>("/record_map_start", 1);
    record_map_end_pub = nh.advertise<std_msgs::Bool>("/record_map_end", 1);
    set_height_by_move_pub = nh.advertise<std_msgs::Float64>("/set_uav_height_by_move", 1);

    //ros subscriber,放到构造函数或run里初始化都是可以的
    log_info_sub = nh.subscribe("/uav_log_info", 2, &RosTopicSendRecv::log_info_sub_callback, this);
    gps_health_sub = nh.subscribe("/uav_gps_health", 1, &RosTopicSendRecv::gps_health_sub_callback, this);
    gps_info_sub = nh.subscribe("/uav_gps_info", 1, &RosTopicSendRecv::gps_info_sub_callback, this);
    uav_vel_info_sub = nh.subscribe("/uav_velocity_info", 1, &RosTopicSendRecv::uav_vel_sub_callback, this);
    height_above_takeoff_sub = nh.subscribe("/height_above_takeoff", 1, &RosTopicSendRecv::height_above_takeoff_sub_callback, this);
    uav_local_pos_sub = nh.subscribe("/uav_local_pos", 1, &RosTopicSendRecv::uav_local_pos_sub_callback, this);
    lidar_nav_rel_pos_sub = nh.subscribe("/lidar_nav_rel_pos", 1, &RosTopicSendRecv::lidar_nav_rel_pos_sub_callback, this);
    lidar_nav_computed_vel_sub = nh.subscribe("/lidar_nav_computed_vel", 1, &RosTopicSendRecv::lidar_nav_computed_vel_sub_callback, this);
    //lidar_nav_odom_sub = nh.subscribe("/lidar_nav_odom_info", 1, &RosTopicSendRecv::lidar_nav_odom_info_callback, this);
    lidar_nav_odom_sub = nh.subscribe("/odom", 1, &RosTopicSendRecv::lidar_nav_odom_info_callback, this);
}

RosTopicSendRecv::~RosTopicSendRecv()
{
}

////////////////////////////////////////////////////////////////////////////
//  publisher
////////////////////////////////////////////////////////////////////////////

void RosTopicSendRecv::pub_takeoff_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    takeoff_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_land_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    land_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_haltManifold_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    halt_manifold_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_gohome_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    gohome_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_resetHeight_cmd(double height)
{
    std_msgs::Float64 ros_msg;
    ros_msg.data = height;
    reset_height_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_activateLidarNav_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    activate_lidar_nav_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_pauseLidarNav_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    pause_lidar_nav_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_resumeLidarNav_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    resume_lidar_nav_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_cancelLidarNav_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    cancel_lidar_nav_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_resetMaxVelocity_cmd(double vel)
{
    std_msgs::Float64 ros_msg;
    ros_msg.data = vel;
    reset_lidar_nav_max_vel_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_change_flight_mode_cmd(UINT8 mode)
{
    std_msgs::UInt8 ros_msg;
    ros_msg.data = mode;
    change_lidar_nav_flight_mode_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_vision_land_start_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    vision_land_start_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_record_start_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    record_map_start_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_record_end_cmd()
{
    std_msgs::Bool ros_msg;
    ros_msg.data = true;
    record_map_end_pub.publish(ros_msg);
}

void RosTopicSendRecv::pub_set_height_by_move_cmd(double height)
{
    std_msgs::Float64 ros_msg;
    ros_msg.data = height;
    set_height_by_move_pub.publish(ros_msg);
}
////////////////////////////////////////////////////////////////////////////
//  callback
////////////////////////////////////////////////////////////////////////////

void RosTopicSendRecv::log_info_sub_callback(const std_msgs::String::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::log_info_sub_callback");
    string log_info = ros_msg->data;
    emit LogInfoSignal(log_info);
}

void RosTopicSendRecv::gps_health_sub_callback(const std_msgs::UInt8::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::gps_health_sub_callback");
    quint8 gps_health = ros_msg->data;
    emit GpsHealthSignal(gps_health);
}

void RosTopicSendRecv::gps_info_sub_callback(const sensor_msgs::NavSatFix::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::gps_info_sub_callback");
    sensor_msgs::NavSatFix gps_info = *ros_msg;
    emit GpsInfoSignal(gps_info);
}

void RosTopicSendRecv::uav_vel_sub_callback(const geometry_msgs::Vector3Stamped::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::uav_vel_sub_callback");
    geometry_msgs::Vector3Stamped uav_vel = *ros_msg;
    emit UavVelocitySignal(uav_vel);
}

void RosTopicSendRecv::height_above_takeoff_sub_callback(const std_msgs::Float32::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::height_above_takeoff_sub_callback");
    double height = ros_msg->data;
    emit HeightAboveTakeoffSignal(height);
}

void RosTopicSendRecv::uav_local_pos_sub_callback(const geometry_msgs::PointStamped::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::uav_local_pos_sub_callback");
    geometry_msgs::PointStamped local_pos = *ros_msg;
    emit UavLocalPosSignal(local_pos);
}

void RosTopicSendRecv::lidar_nav_rel_pos_sub_callback(const geometry_msgs::PointStamped::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::lidar_nav_rel_pos_sub_callback");
    geometry_msgs::PointStamped rel_pos = *ros_msg;
    emit LidarNavRelPosSignal(rel_pos);
}

void RosTopicSendRecv::lidar_nav_computed_vel_sub_callback(const geometry_msgs::Vector3Stamped::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::lidar_nav_computed_vel_sub_callback");
    geometry_msgs::Vector3Stamped vel = *ros_msg;
    emit LidarNavComputedVelSignal(vel);
}

void RosTopicSendRecv::lidar_nav_odom_info_callback(const nav_msgs::Odometry::ConstPtr &ros_msg)
{
    ROS_INFO("in RosTopicSendRecv::lidar_nav_odom_info_callback");
    nav_msgs::Odometry odom = *ros_msg;
    emit LidarNavOdomSignal(odom);
}

////////////////////////////////////////////////////////////////////////////
//  run
////////////////////////////////////////////////////////////////////////////

void RosTopicSendRecv::run()
{
    ROS_INFO("in RosTopicSendRecv::run");
    sleep(1);

    ROS_INFO("ready to ros spin");
    while (ros::ok())
    {
        ros::spinOnce();
        //ros::Rate(1).sleep();
    }
}
