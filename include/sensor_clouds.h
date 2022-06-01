#ifndef SENSOR_CLOUDS_H
#define SENSOR_CLOUDS_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <queue>
#include <mutex>
#include <iostream>

/**
 * LidarCloud class is meant to keep track/handle a single lidar's input to CloudNode
 */
class LidarCloud
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointXYZRGBNormal PointTypeNorms;

    LidarCloud(const std::string _frame_id, const std::string _to_frame, const std::string _pcl2_topic, ros::NodeHandle* _p_nh, tf2_ros::Buffer* _p_tf2_buffer, bool publish_solo_msgs = false);
    
    void cleaner(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);
    
    void printCloudSpecs(const pcl::PointCloud<PointType>::ConstPtr &cloud_in);

    void handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

private:
    ros::NodeHandle* nh;
    ros::Subscriber sub_raw_cloud;
    ros::Publisher pub_solo_cloud, pub_to_icp;
    
    std::string frame_id, to_frame;

    tf2_ros::Buffer* p_tf2_buffer;
};

/**
 * RadarCloud is meant to track/handle a single radar's input to CloudNode
 */
class RadarCloud
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointXYZRGBNormal PointTypeNorms;

    RadarCloud(const std::string _frame_id, const std::string _to_frame, const std::string _pcl2_topic, ros::NodeHandle* _p_nh, tf2_ros::Buffer* _p_tf2_buffer, pcl::PointCloud<PointType>::Ptr _fused_cloud, bool publish_solo_msgs = true);

    void cleaner(const pcl::PointCloud<PointType> &cloud_in, pcl::PointCloud<PointType> &cloud_out);

    void handler(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
private:
    ros::NodeHandle* nh;
    ros::Subscriber sub_raw_cloud;
    ros::Publisher pub_solo_cloud;
    
    std::string frame_id, to_frame;

    tf2_ros::Buffer* p_tf2_buffer;
    pcl::PointCloud<PointType>::Ptr p_fused_cloud;
};

#endif  // SENSOR_CLOUDS_H
