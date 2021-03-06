
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "drone_common/ros_util.h"

// Odom global message
nav_msgs::Odometry odom_message;

/**
 * Callback which gets the Odometry information over MAVROS
 */
void odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_message = *msg;    
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odometry_transform_node");
    ros::NodeHandle nh;

    //Parameters 
    double refresh_rate = GetParam<double>( nh, ros::this_node::getName() + "/refresh_rate", 30.0 );

    // Hardcoded subscriber for the odometry message generated by MAVROS
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odomCB);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();

    //Set up our rate based on input parameters
    ros::Rate r(refresh_rate);
    while(nh.ok())
    {
        // Check for incoming messages
        ros::spinOnce();

        // Get current timestamp for the transform message
        current_time = ros::Time::now();
        
        // Publish transform based on incoming odom message
        // generated by MAVROS
        geometry_msgs::TransformStamped odom_trans;
        tf::Quaternion q_norm;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = odom_message.pose.pose.position.x;
        odom_trans.transform.translation.y = odom_message.pose.pose.position.y;
        odom_trans.transform.translation.z = odom_message.pose.pose.position.z;

        //TODO: Figure out way to check whether the incoming quaternion is valid
        // Tried doing it with normalize fuction from tf::Quaternion but it did
        // not work.
        odom_trans.transform.rotation.x = odom_message.pose.pose.orientation.x;    
        odom_trans.transform.rotation.y = odom_message.pose.pose.orientation.y;    
        odom_trans.transform.rotation.z = odom_message.pose.pose.orientation.z;    
        odom_trans.transform.rotation.w = odom_message.pose.pose.orientation.w;

        odom_broadcaster.sendTransform(odom_trans);

        r.sleep();    
    }
}
