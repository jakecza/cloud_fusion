#include "scan_matching.h"
#include "sensor_clouds.h"
#include "tic_toc.h"
#include <pcl/features/normal_3d.h>

#define P2POINT

ros::Subscriber sub_radar_from_sensor_cloud, sub_lidar_from_sensor_cloud;

ros::Publisher pub_pose, pub_mapped;

uint count = 0;
Eigen::Matrix4d currentPose, aprioriPose;

#ifdef P2POINT
    // P2Point ICP
    pcl::IterativeClosestPoint<RadarCloud::PointType, RadarCloud::PointType> icp;
    pcl::PointCloud<RadarCloud::PointType>::Ptr current_rCloud ( new pcl::PointCloud<RadarCloud::PointType> () );
    pcl::PointCloud<RadarCloud::PointType>::Ptr postTF_rCloud ( new pcl::PointCloud<RadarCloud::PointType> () );
    pcl::PointCloud<RadarCloud::PointType>::Ptr mapped_rCloud ( new pcl::PointCloud<RadarCloud::PointType> () );
#endif

#ifdef P2PLANE
    // P2Plane ICP
    pcl::IterativeClosestPointWithNormals<RadarCloud::PointTypeNorms, RadarCloud::PointTypeNorms> icp;
    pcl::PointCloud<RadarCloud::PointType>::Ptr current_bare_rCloud ( new pcl::PointCloud<RadarCloud::PointType> () );
    pcl::PointCloud<RadarCloud::PointType>::Ptr mapped_bare_rCloud ( new pcl::PointCloud<RadarCloud::PointType> () );
    pcl::PointCloud<RadarCloud::PointTypeNorms>::Ptr current_rCloud ( new pcl::PointCloud<RadarCloud::PointTypeNorms> () );
    pcl::PointCloud<RadarCloud::PointTypeNorms>::Ptr postTF_rCloud ( new pcl::PointCloud<RadarCloud::PointTypeNorms> () );
    pcl::PointCloud<RadarCloud::PointTypeNorms>::Ptr mapped_rCloud ( new pcl::PointCloud<RadarCloud::PointTypeNorms> () );
#endif


void addNormals(pcl::PointCloud<RadarCloud::PointType>::Ptr bare_cloud, pcl::PointCloud<RadarCloud::PointTypeNorms>::Ptr norm_cloud){
    pcl::PointCloud<pcl::Normal>::Ptr normals ( new pcl::PointCloud<pcl::Normal> );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr searchTree (new pcl::search::KdTree<pcl::PointXYZ>);
    searchTree->setInputCloud ( bare_cloud );

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud ( bare_cloud );
    normalEstimator.setSearchMethod ( searchTree );
    normalEstimator.setKSearch ( 15 );
    normalEstimator.compute ( *normals );
    
    pcl::concatenateFields( *bare_cloud, *normals, *norm_cloud );
}


void runRadarICP(const sensor_msgs::PointCloud2ConstPtr &cloud_msg){
    TicToc t_icp;

    #ifdef P2POINT
        pcl::fromROSMsg(*cloud_msg, *current_rCloud);
    #endif
    #ifdef P2PLANE
        pcl::fromROSMsg(*cloud_msg, *current_bare_rCloud);
        addNormals(current_bare_rCloud, current_rCloud);
        std::vector<int> indices;
        pcl::removeNaNNormalsFromPointCloud(*current_rCloud, *current_rCloud, indices);
    #endif

    if(!mapped_rCloud->empty() && !current_rCloud->empty()){
        count += 1;

        // Rotate current scan based on current pose
        #ifdef P2POINT
            pcl::transformPointCloud(*current_rCloud, *current_rCloud, currentPose);
        #endif
        #ifdef P2PLANE
            pcl::transformPointCloud(*current_bare_rCloud, *current_bare_rCloud, currentPose);
            pcl::transformPointCloudWithNormals(*current_rCloud, *current_rCloud, currentPose);
        #endif

        // Find delta TF for current scan
        icp.setInputSource(current_rCloud);
        icp.setInputTarget(mapped_rCloud);
        icp.align(*postTF_rCloud);

        // std::cout << "Post align = " << t_icp.toc() << std::endl;

        if(icp.hasConverged())
        {   
            // Find dTF from scan in world frame to mapped_cloud in world frame
            Eigen::Matrix4d dTF = (icp.getFinalTransformation()).cast <double> ();

            // Adjust current scan based on corrected dTF
            #ifdef P2POINT
                pcl::transformPointCloud(*current_rCloud, *current_rCloud, dTF);
            #endif
            #ifdef P2PLANE
                pcl::transformPointCloud(*current_bare_rCloud, *current_bare_rCloud, dTF);
                pcl::transformPointCloudWithNormals(*current_rCloud, *current_rCloud, dTF);
            #endif

            // Add current scan to mapped history cloud
            *mapped_rCloud += *current_rCloud;
            #ifdef P2PLANE
                *mapped_bare_rCloud += *current_bare_rCloud;
            #endif

            currentPose = currentPose*dTF;

            Eigen::Quaterniond currQ(Eigen::Matrix3d(currentPose.topLeftCorner(3,3)));
            Eigen::Vector3d currP = currentPose.topRightCorner(3,1);

            nav_msgs::Odometry odom_out;
            odom_out.header.frame_id = "map_fusion";
            odom_out.child_frame_id = "base_link";
            odom_out.pose.pose.orientation = tf2::toMsg(currQ);
            odom_out.pose.pose.position = tf2::toMsg(currP);
            odom_out.header.stamp = cloud_msg->header.stamp;
            pub_pose.publish(odom_out);

            sensor_msgs::PointCloud2 map_msg;
            #ifdef P2POINT
                pcl::toROSMsg(*mapped_rCloud, map_msg);
            #endif
            #ifdef P2PLANE
                pcl::toROSMsg(*mapped_bare_rCloud, map_msg);
            #endif
            map_msg.header.frame_id = "map_fusion";
            pub_mapped.publish(map_msg);

        } else {
            std::cout << "Radar ICP not converged" << std::endl;
        }

    } else {
        if( mapped_rCloud->empty() ){
            *mapped_rCloud = *current_rCloud;
            #ifdef P2PLANE
                *mapped_bare_rCloud = *current_bare_rCloud;
            #endif
        }

        ROS_WARN_THROTTLE(10, "ICP algorithm does not have access to a radar apriori cloud yet. Waiting for lidar_fused_cloud history to accumulate...");
    }
    current_rCloud->clear();
    postTF_rCloud->clear();
    #ifdef P2PLANE
        current_bare_rCloud->clear();
    #endif
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "scan_matching");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    // Bring in ROS params to choose filter type, ICP type, etc

    sub_radar_from_sensor_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_node/fused_cloud", 100, runRadarICP);
    pub_pose = nh.advertise<nav_msgs::Odometry>("/scan_matching/odom", 10);
    pub_mapped = nh.advertise<sensor_msgs::PointCloud2>("/scan_matching/mapped_pts", 10);

    currentPose = Eigen::Matrix4d::Identity();
    aprioriPose = Eigen::Matrix4d::Identity();

    ros::spin();
    ros::waitForShutdown();

    return 0;
}
