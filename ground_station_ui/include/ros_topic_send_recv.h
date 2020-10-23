#ifndef ROS_TOPIC_SEND_RECV_H
#define ROS_TOPIC_SEND_RECV_H

#include "typeinc.h"
#include <ros/ros.h>
#include <QObject>
#include <QThread>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

class RosTopicSendRecv : public QThread
{
    Q_OBJECT
public:
    RosTopicSendRecv(QObject *parent = nullptr);    
    virtual ~RosTopicSendRecv();    
    void pub_takeoff_cmd();
    void pub_land_cmd();
    void pub_haltManifold_cmd();
    void pub_gohome_cmd();
    void pub_resetHeight_cmd(double height);
    void pub_activateLidarNav_cmd();
    void pub_pauseLidarNav_cmd();
    void pub_resumeLidarNav_cmd();
    void pub_cancelLidarNav_cmd();
    void pub_resetMaxVelocity_cmd(double vel);
    void pub_change_flight_mode_cmd(UINT8 mode);
    void pub_vision_land_start_cmd();
    void pub_record_start_cmd();
    void pub_record_end_cmd();
    void pub_set_height_by_move_cmd(double height);
protected:
    void run();

signals:
    void LogInfoSignal(const string loginfo);
    void GpsHealthSignal(const quint8 gps_health);
    void GpsInfoSignal(const sensor_msgs::NavSatFix gpsinfo);
    void UavVelocitySignal(const geometry_msgs::Vector3Stamped vel);
    void HeightAboveTakeoffSignal(const double height);
    void UavLocalPosSignal(const geometry_msgs::PointStamped pos);
    void LidarNavRelPosSignal(const geometry_msgs::PointStamped pos);
    void LidarNavComputedVelSignal(const geometry_msgs::Vector3Stamped vel);
    void LidarNavOdomSignal(const nav_msgs::Odometry odom);
    
private:
    ros::NodeHandle nh;
    //publisher   
    ros::Publisher takeoff_pub;
    ros::Publisher land_pub;
    ros::Publisher gohome_pub;
    ros::Publisher halt_manifold_pub;
    ros::Publisher reset_height_pub;
    ros::Publisher activate_lidar_nav_pub;
    ros::Publisher pause_lidar_nav_pub;
    ros::Publisher resume_lidar_nav_pub;
    ros::Publisher cancel_lidar_nav_pub;
    ros::Publisher reset_lidar_nav_max_vel_pub;
    ros::Publisher change_lidar_nav_flight_mode_pub;
    ros::Publisher vision_land_start_pub;
    ros::Publisher record_map_start_pub;
    ros::Publisher record_map_end_pub;
    ros::Publisher set_height_by_move_pub;
public:    
    //subscribe
    ros::Subscriber log_info_sub;
    ros::Subscriber gps_health_sub;
    ros::Subscriber gps_info_sub;
    ros::Subscriber uav_vel_info_sub;
    ros::Subscriber height_above_takeoff_sub;
    ros::Subscriber uav_local_pos_sub;
    ros::Subscriber lidar_nav_rel_pos_sub;
    ros::Subscriber lidar_nav_computed_vel_sub;
    ros::Subscriber lidar_nav_odom_sub;
    std::string map_dir;
    void log_info_sub_callback(const std_msgs::String::ConstPtr& ros_msg);
    void gps_info_sub_callback(const sensor_msgs::NavSatFix::ConstPtr& ros_msg); 
    void gps_health_sub_callback(const std_msgs::UInt8::ConstPtr& ros_msg);
    void height_above_takeoff_sub_callback(const std_msgs::Float32::ConstPtr& ros_msg);
    void uav_vel_sub_callback(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg);
    void uav_local_pos_sub_callback(const geometry_msgs::PointStamped::ConstPtr& ros_msg);
    void lidar_nav_rel_pos_sub_callback(const geometry_msgs::PointStamped::ConstPtr& ros_msg);
    void lidar_nav_computed_vel_sub_callback(const geometry_msgs::Vector3Stamped::ConstPtr& ros_msg);
    void lidar_nav_odom_info_callback(const nav_msgs::Odometry::ConstPtr& ros_msg);
};

#endif