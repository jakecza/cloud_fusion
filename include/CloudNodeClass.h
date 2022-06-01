#ifndef CLOUD_NODE_CLASS_H
#define CLOUD_NODE_CLASS_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <queue>

#include "sensor_clouds.h"

//! CloudNode Constructor
class CloudNode
{
public:
    CloudNode(ros::NodeHandle& _nh, ros::NodeHandle& _nh_priv);
    ~CloudNode() {};

    bool init();
    void spin();
    void publish_fused_cloud(const ros::TimerEvent& e = ros::TimerEvent());

    // ROS objects
    ros::NodeHandle nh, nh_priv;
    ros::Publisher pub_fused_cloud, pub_mapped_cloud;
    ros::Timer timer_fused_cloud;

    // TF2 & PCL objects
    tf2_ros::Buffer *p_tf2_buffer;
    tf2_ros::TransformListener *p_tf_listener;

    // ROS params
    std::string base_frame, world_frame;
    std::string radar_cluster_frame, lidar_cluster_frame;

    // State params
    std::vector<boost::shared_ptr<RadarCloud>> active_radar_list;
    std::vector<boost::shared_ptr<LidarCloud>> active_lidar_list;

    pcl::PointCloud<RadarCloud::PointType>::Ptr p_fused_cloud, p_apriori_cloud, p_mapped_cloud, p_tfed_cloud;
    pcl::IterativeClosestPoint<RadarCloud::PointType, RadarCloud::PointType> icp_radar;

protected:

private:


};


#endif  // CLOUD_NODE_CLASS_H
