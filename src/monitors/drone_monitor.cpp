/********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Airlitix
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Airlitix nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Carlos Diaz Alvarenga & Andrew Dobson */
/* Node to control the drone with velocity commands (output from the navigation stack) at a given height*/
#include <tf2_ros/static_transform_broadcaster.h>
#include "drone.h"
#include "ros_util.h"
#include <mutex>

//Some Global Checking information
ros::Time last_odom_time(0);
double odom_dropout_threshold;
std::mutex odom_time_mx;

bool map_transform_set;
bool odom_live;
bool simulation;
bool using_mapping;
nav_msgs::Odometry odom_message;

//Depending on the firmware, we need to check for different modes
std::string cc_control_mode;

/**
 * Callback for reading the odom time
 */
void odom_check(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_time_mx);
    last_odom_time = msg->header.stamp;
    odom_message = *msg;
    // ROS_WARN("WE HAVE ODOM, ODOM IS HERE!!!!!!!!!");
}

/**
 * Drone monitor node used to monitor the status of the drone.
 * Also ued to arm and set the mode of the drone.
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "drone_monitor");
    ros::NodeHandle nh;
    
    ROS_WARN("[drone_monitor] STARTING...");

    //Some members we'll use to track things
    drone_interface::Drone drone;
    odom_live = false;
    map_transform_set = false;
    odom_dropout_threshold = GetParam<double>( nh, ros::this_node::getName() + "/odom_dropout_threshold", 0.1 );
    double update_gap = 99.0;
    simulation = GetParam<bool>( nh, "/use_sim_time" );
    using_mapping = GetParam<bool>( nh, "/use_mapping", false );
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    //Alright, we need to read what firmware we're running for
    std::string fw = GetParam<std::string>( nh, ros::this_node::getName() + "/firmware", "-" );
    if( fw == "px4" )
    {
        cc_control_mode = "OFFBOARD";
    }
    else if( fw == "apm" )
    {
        cc_control_mode = "GUIDED";
    }
    else
    {
        ROS_FATAL_STREAM("Unknown Firmware type [" << fw << "] : Aborting...");
        exit(4);
    }
    ROS_WARN_STREAM("Proceeding with firmware [" << fw << "]");

    //If we are using a mapping solution, then we will assume our map transformation is set
    if( using_mapping )
    {
        map_transform_set = true;
    }

    //We need to set up subscribers which check if things are healthy
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 20, odom_check);

    // First: Wait for FCU connection
    ros::Rate rate(2.0);
    while( ros::ok() && !drone.getCurrentState().connected )
    {
        ros::spinOnce();
        ROS_WARN("Waiting for FCU connection...");
        drone.reportCurrentState();
        rate.sleep();
    }
    ROS_INFO("Flight Controller Connected!");

    while( ros::ok() )
    {
        //For the sake of consistency, we have to wait until we have a position 
        //  estimation from the drone before attempting to arm (need to set map transform)
        while( ros::ok() && !odom_live )
        {
            //do the check for odom liveness
            ros::spinOnce();
            { //Only lock the mutex for when we are doing the check
                std::lock_guard<std::mutex> lock(odom_time_mx);
                update_gap = (ros::Time::now() - last_odom_time).toSec();
                odom_live = update_gap < odom_dropout_threshold;
            }
            ROS_WARN("Waiting for odometry information...");

            //Now, if we've gotten odometry information, but haven't set up our
            // map transformation
            if( odom_live && !map_transform_set )
            {
                //Get the orientation information
                double x = odom_message.pose.pose.orientation.x;    
                double y = odom_message.pose.pose.orientation.y;    
                double z = odom_message.pose.pose.orientation.z;    
                double w = odom_message.pose.pose.orientation.w;
                //Then I need to extract the transform from this...
                tf::Quaternion q(x,y,z,w);
                tf::Vector3 axis = q.getAxis();
                double angle = q.getAngle();
                //I can do some error checking to see if the axis is off... but then what?
                //Assuming the axis isn't too far off, I should be able to build the transform.
                tf::Quaternion map_q( tf::Vector3(0,0,1), angle );
                // tf::Transform map_odom_static( map_q, tf::Vector3(0,0,0) );

                geometry_msgs::TransformStamped static_transformStamped;
                static_transformStamped.header.stamp = ros::Time::now();
                static_transformStamped.header.frame_id = "map";
                static_transformStamped.child_frame_id = "odom";
                
                static_transformStamped.transform.translation.x = 0;
                static_transformStamped.transform.translation.y = 0;
                static_transformStamped.transform.translation.z = 0;
                static_transformStamped.transform.rotation.x = map_q.x();
                static_transformStamped.transform.rotation.y = map_q.y();
                static_transformStamped.transform.rotation.z = map_q.z();
                static_transformStamped.transform.rotation.w = map_q.w();

                static_broadcaster.sendTransform(static_transformStamped);

                map_transform_set = true;
            }

            rate.sleep();
        }

        //Arm drone if in simulation, otherwise just wait
        while (ros::ok() && !drone.getCurrentState().armed)
        {
            ROS_WARN("Waiting for drone to arm...");
            if( simulation )
            {
                ROS_INFO("Sending arm command");
                drone.arm();
            }
            ros::spinOnce();
            rate.sleep();
        }
        ROS_INFO("Drone Armed!");
    
        // Set Mode to our companion computer control mode
        while( ros::ok() && drone.getCurrentState().mode != cc_control_mode )
        {
            if( simulation )
            {
                drone.setMode(cc_control_mode);
            }
            ros::spinOnce();
            ROS_WARN_STREAM("Waiting for control override: current control mode: [" << drone.getCurrentState().mode << "]");
            rate.sleep();
        }
        ROS_INFO_STREAM("Drone mode set to " << cc_control_mode << "!");

        //Then, sit and idle, checking if the status is okay
        ros::Rate idlerate(30.0);
        update_gap = 0.0;
        while( ros::ok() && drone.getCurrentState().mode == cc_control_mode && drone.getCurrentState().armed && odom_live )
        {
            ros::spinOnce();
            { //Only lock the mutex for when we are doing the check
                std::lock_guard<std::mutex> lock(odom_time_mx);
                update_gap = (ros::Time::now() - last_odom_time).toSec();
                odom_live = update_gap < odom_dropout_threshold;
            }
            idlerate.sleep();
        }

        //If odometry information has been lost, we are in srs trouble
        if( !odom_live )
        {
            ROS_WARN_STREAM("Odometry information has been lost! :: Gap [" << update_gap << "]");
        }
    }

    //TODO: If the status here goes awol, what should our behavior be?

    return 0;
}
