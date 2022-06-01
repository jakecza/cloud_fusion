/* 
The CloudNode ROS node is meant to:
    - create Cloud objects that have functions to clean and handle incoming PCL2 msgs
    - transform those incoming cloud msgs to the world frame based on a current odom estimate
    - fuse the transformed cloud msgs into a single cloud to be published
*/

#include "CloudNodeClass.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cloud_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    CloudNode cloud_node(nh, nh_priv);

    cloud_node.init();
    cloud_node.spin();

    ros::waitForShutdown();

    return 0;
}
