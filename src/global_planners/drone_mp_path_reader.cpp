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
/* Node to control the drone by sending "paths" to the firmware.
 * Position setpoints are used based on the EFK/Optical Flow data. */

#include "drone.h"

#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Twist.h>

#include "visualization/rviz_util.h"
#include "drone_msgs/commandPath.h"

#include "ros_util.h"

#include <mutex>
#include <fstream>

// A path read in from a file
drone_msgs::commandPath read_path;

//Mutex for the odometry information
std::mutex odom_mx;
// Odom global message
nav_msgs::Odometry odom_message;

// Global boolean for whether we are doing RTH
bool return_to_home = true;

/**
 * Callback for reading the actual odom information
 */
void odomCB(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mx);
    odom_message = *msg;
}


/**
 *
 */
void send_path( ros::NodeHandle& nh )
{
    ros::ServiceClient path_client = nh.serviceClient<drone_msgs::commandPath>("/airlitix/follow_path");

    if(ros::ok())
    {
        if( !path_client.call( read_path ) )
        {
            ROS_ERROR("Unable to contact path follower!");
        }
    }
    else
    {
        ROS_ERROR("ROS is not running, naive navigation exiting.");
    }
}

/**
 *
 */
bool load_path( std::string filename )
{
    std::ifstream file_reader;
    file_reader.open( filename );

    //First, the file needs to specify how many waypoints we want
    unsigned nr_waypoints = 0;
    file_reader >> nr_waypoints;
    read_path.request.waypoints.resize( nr_waypoints );

    //If we can't load the path, we need to abort
    if( !file_reader.good() )
    {
        return false;
    }

    //Now we just need to read (x,y,z) triples
    double x; double y; double z;
    for( unsigned i=0; i<nr_waypoints; ++i )
    {
        file_reader >> x >> y >> z;
        read_path.request.waypoints[i].pose.position.x = x;
        read_path.request.waypoints[i].pose.position.y = y;
        read_path.request.waypoints[i].pose.position.z = z;
        read_path.request.waypoints[i].cmd = "w";
    }

    //If we are going to return to home, we need to insert some stuff
    if( return_to_home )
    {
        read_path.request.waypoints.resize( nr_waypoints +2 );
        nr_waypoints += 2;
        // Second-to-last waypoint should just get us above home
        read_path.request.waypoints[nr_waypoints-2].pose.position.x = 0;
        read_path.request.waypoints[nr_waypoints-2].pose.position.y = 0;
        read_path.request.waypoints[nr_waypoints-2].pose.position.z = read_path.request.waypoints[0].pose.position.z;
        read_path.request.waypoints[nr_waypoints-2].cmd = "w";
        // Final waypoint should make us land
        read_path.request.waypoints[nr_waypoints-1].cmd = "l";
    }

    //DEBUG
    auto& wp = read_path.request.waypoints;
    for( unsigned i=0; i<nr_waypoints; ++i )
    {
        ROS_INFO("[%s] : %f %f %f", wp[i].cmd.c_str(), wp[i].pose.position.x, wp[i].pose.position.y, wp[i].pose.position.z);
    }


    return true;
}

/**
 *
 */
void transform_path()
{
    double hnw;
    double x, y, z;
    //First, let's read in the odom information to find our orientation.
    {
        std::lock_guard<std::mutex> lock(odom_mx);
        //Get position information
        x = odom_message.pose.pose.position.x;
        y = odom_message.pose.pose.position.y;
        z = odom_message.pose.pose.position.z;
        //and compute that angle
        double qx = odom_message.pose.pose.orientation.x;
        double qy = odom_message.pose.pose.orientation.y;
        double qz = odom_message.pose.pose.orientation.z;
        double qw = odom_message.pose.pose.orientation.w;

        hnw = 2 * acos( -qw );
    }
    //Now for each of those points, we just need to rotate the vector by that heading.
    for( unsigned i=0; i<read_path.request.waypoints.size(); ++i )
    {
        //Compute the angle relative to the robot center
        double dx = read_path.request.waypoints[i].pose.position.x - x;
        double dy = read_path.request.waypoints[i].pose.position.y - y;
        //magnitude of the point
        double mag = sqrt( dx * dx + dy * dy );
        //Angle fo the point
        double ang = atan2( dy, dx );

        //Now for the new point info, update this angle
        ang += hnw;
        while( ang > 6.28318530718 )
        {
            ang -= 6.28318530718;
        }

        //And the point is in that new direction
        read_path.request.waypoints[i].pose.position.x = x + mag*cos(ang);
        read_path.request.waypoints[i].pose.position.y = y + mag*sin(ang);
    }
}

/**
 *
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "naive_nav_node");

    //Some members for interfacing with ROS and the drone
    drone_interface::Drone drone;
    ros::NodeHandle nh;

    //In general, a motion planner is going to need to have odometry information
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 20, odomCB);

    //Let's find out what file we have to load
    std::string filename = GetParam<std::string>(nh, ros::this_node::getName() + "/path_file");
    return_to_home = GetParam<bool>( nh, ros::this_node::getName() + "/return_to_home", true );
    bool do_transform = GetParam<bool>( nh, ros::this_node::getName() + "/transform_path", false );

    //We're going to wait until we hear that the drone is armed
    ros::Rate armcheck(1.0);
    while (ros::ok() && !drone.getCurrentState().armed )
    {
        ros::spinOnce();
        armcheck.sleep();
    }
    ROS_INFO("Drone is armed and Odom data up, proceeding to send waypoints");

    //Attempt to read and send a path
    if( load_path( filename ) )
    {
        //Then, we need to check our odom information, get a sense of where we are, and transform the path
        if( do_transform )
        {
            transform_path();
        }
        //Then, we'll send the transformed path
        send_path(nh);
        return 0;
    }

    return -2;
}
