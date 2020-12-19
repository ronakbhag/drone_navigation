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

#ifndef PATH_FOLLOW_SERVICES_H
#define PATH_FOLLOW_SERVICES_H

#include "drone.h"

#include <tf/transform_listener.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include <deque>
#include <mutex>

#include "drone_msgs/commandPath.h"
#include "drone_msgs/commandSetpoint.h"

//Mutices for the path following
std::mutex nav_stack_mx;

//Setpoint memory, so we can follow the segment rather than just the point
tf::Transform LAST_SETPOINT;
tf::Transform NEXT_SETPOINT;
tf::Transform TARGET;

//Drone interface class
drone_interface::Drone* drone;

//Some parameters for path following, might need to be input at some point
double LOOK_AHEAD;
double DISTANCE_THRESHOLD;
double PATH_FOLLOW_RATE;

//Some dirty debugging for initialization
bool recieved_target = false;

// Modality of control: whether we use the avoidance package
bool use_avoidance = false;
bool use_vision = true;
bool self_start = false;
bool autonomous_control = false;

// Timing and delays for practical stability
double loiter_duration = 0.0;
bool arrived_at_checkpoint = false;
ros::Time checkpoint_arrival_time;
double initial_wait_duration = 0.0;
bool initialization_done = false;
ros::Time path_arrival_time;

// Global message information
nav_msgs::Odometry odom_message;
geometry_msgs::PoseStamped pose_message;
geometry_msgs::PoseStamped vision_message;

// Global distance tracking things
double bb, cc, dd, d, dist;

// Global variable destination so that the thread can read 
// from this and change the desired setpoint accordingly
mavros_msgs::PositionTarget vel_flight_controller;
std::deque< drone_msgs::waypoint > output_nav_stack;
geometry_msgs::PoseStamped goal_flight_controller;

//Global Publisher and Subscriber
ros::Subscriber setpoint_sub;
ros::Publisher setpointRawPub;
ros::Publisher clickedGoalPub;
ros::Publisher visualization_marker_pub;


/**
 * Utility Distance function: computes distance between setpoints
 */
void compute_setpoint_distance()
{
    dd= ( LAST_SETPOINT.getOrigin()[0] - NEXT_SETPOINT.getOrigin()[0] )*( LAST_SETPOINT.getOrigin()[0] - NEXT_SETPOINT.getOrigin()[0] ) + 
        ( LAST_SETPOINT.getOrigin()[1] - NEXT_SETPOINT.getOrigin()[1] )*( LAST_SETPOINT.getOrigin()[1] - NEXT_SETPOINT.getOrigin()[1] ) + 
        ( LAST_SETPOINT.getOrigin()[2] - NEXT_SETPOINT.getOrigin()[2] )*( LAST_SETPOINT.getOrigin()[2] - NEXT_SETPOINT.getOrigin()[2] );
    d = sqrt( dd );
}

/**
 * A simple assignment helper
 */
void simple_vec_assign( std::vector<double>& dest, std::vector<double>& source )
{
    for( unsigned i=0; i<dest.size(); ++i )
    {
        dest[i] = source[i];
    }
}

/**
 * Utility:
 */
void pose_to_transform( tf::Transform& dest, geometry_msgs::Pose& source )
{
    dest.setOrigin( tf::Vector3( source.position.x, source.position.y, source.position.z ) );
    dest.setRotation( tf::Quaternion( source.orientation.x, source.orientation.y, source.orientation.z, source.orientation.w ) );
}

/**
 * Utility:
 */
double tf_pose_sq_dist( tf::Transform& trans, geometry_msgs::Pose& pose )
{
    return ( trans.getOrigin()[0] - pose.position.x )*( trans.getOrigin()[0] - pose.position.x ) + ( trans.getOrigin()[1] - pose.position.y )*( trans.getOrigin()[1] - pose.position.y ) + ( trans.getOrigin()[2] - pose.position.z )*( trans.getOrigin()[2] - pose.position.z );
}

/**
 * Service to go to a single setpoint
 */
bool go_to_point( drone_msgs::commandSetpoint::Request  &req,
                  drone_msgs::commandSetpoint::Response &res)
{
    std::lock_guard<std::mutex> lock(nav_stack_mx);

    output_nav_stack.clear();
    recieved_target = true;

    pose_to_transform( NEXT_SETPOINT, req.target );
    (use_vision ? pose_to_transform( LAST_SETPOINT, vision_message.pose ) : pose_to_transform( LAST_SETPOINT, odom_message.pose.pose ));

    compute_setpoint_distance();

    path_arrival_time = ros::Time::now();
    res.accepted = true;
    return true;
}

/**
 * Service for following a path
 */
bool follow_path( drone_msgs::commandPath::Request  &req,
                  drone_msgs::commandPath::Response &res)
{
    std::lock_guard<std::mutex> lock(nav_stack_mx);

    output_nav_stack.clear();

    //If we were given an empty message, then we failed
    if( req.waypoints.size() < 1 )
    {
        res.accepted = false;
        return false;
    }
    recieved_target = true;
    //Set the waypoints
    pose_to_transform( NEXT_SETPOINT, req.waypoints[0].pose );
    (use_vision ? pose_to_transform( LAST_SETPOINT, pose_message.pose ) : pose_to_transform( LAST_SETPOINT, odom_message.pose.pose ));

    //Need to always compute distance information or we have no idea how to make progress
    compute_setpoint_distance();

    for( unsigned i=1; i<req.waypoints.size(); ++i )
    {
        output_nav_stack.push_back( req.waypoints[i] );
    }

    path_arrival_time = ros::Time::now();
    res.accepted = true;
    return true;
}

/**
 * Callback for reading the odom->map transform
 */
void localization_lookup( ros::NodeHandle& nh, tf::TransformListener& tf_listener, tf::StampedTransform& odom_map_tf )
{
    if( nh.ok() )
    {
        try
        {
            tf_listener.lookupTransform("/odom", "/map", ros::Time(0), odom_map_tf);
        }
        catch(std::runtime_error e)
        {

        }
    }
}

/**
 * Path-Following Callback: Reads odom information and sets TARGET to move toward
 */
void pathFollowCallback()
{
    ros::Rate rate(PATH_FOLLOW_RATE);

    ros::NodeHandle nh;
    ros::Subscriber vis_sub;
    geometry_msgs::Pose gt_pose;

    //TF listener params
    tf::TransformListener tf_listener;
    tf::StampedTransform odom_map_tf;

    while( ros::ok() )
    {
        //Spin
        ros::spinOnce();

        if( !initialization_done && (ros::Time::now() - path_arrival_time) > ros::Duration(initial_wait_duration) )
        {
            initialization_done = true;
        }

        if( recieved_target && initialization_done )
        {
            // Get the latest position information
            gt_pose = drone->getPose();

            // We'll need setpoint transforms
            tf::Transform ls_odom = LAST_SETPOINT;
            tf::Transform ns_odom = NEXT_SETPOINT;
            //Apply transformations, if applicable
            if( !use_vision )
            {
                localization_lookup( nh, tf_listener, odom_map_tf );
                // ROS_INFO_STREAM("Inverse transform [" << odom_map_tf.getOrigin()[0] << ", " << odom_map_tf.getOrigin()[1] << ", " << odom_map_tf.getOrigin()[2] << "]");

                // Want to get the setpoints in odom frame
                ls_odom = odom_map_tf * LAST_SETPOINT;
                ns_odom = odom_map_tf * NEXT_SETPOINT;
            }

            //Compute some squared distances
            cc = tf_pose_sq_dist( ls_odom, gt_pose );
            bb = tf_pose_sq_dist( ns_odom, gt_pose );

            double x = (0.5/d)*(cc + dd - bb);

            //Now we need to see if the distance is to the segment or one of the endpoints
            if( x < 0 )
                dist = sqrt( cc );
            else if( x > d )
                dist = sqrt( bb );
            else
                dist = sqrt( cc - (x*x) );

            //We should also update our target: We want to move along the segment,
            //but if we are very far away from the segment, we want to just move 
            //towards the segment first
            double delta = (1 - (dist/DISTANCE_THRESHOLD));
            if(delta < 0)
                delta = 0;
            double forward_dist = LOOK_AHEAD * delta;

            // Compute how far along we are on the segment
            double t = ((x + forward_dist)/d);
            if( t < 0 ) //If we are well behind the segment, let's just try to get to the beginning of it
                TARGET = ls_odom;
            else if( t >= 1 ) //If we are close to the setpoint, let's just go there
                TARGET = ns_odom;
            else //Otherwise, we need to try to get onto the segment
                TARGET.setOrigin( tf::Vector3( ls_odom.getOrigin()[0] + t * ( ns_odom.getOrigin()[0] - ls_odom.getOrigin()[0] ), ls_odom.getOrigin()[1] + t * ( ns_odom.getOrigin()[1] - ls_odom.getOrigin()[1] ), ls_odom.getOrigin()[2] + t * ( ns_odom.getOrigin()[2] - ls_odom.getOrigin()[2] ) ) );


            //Also, if we're near the setpoint, it's time to move to the next
            if( t >= 1 + (0.5*forward_dist)/d && output_nav_stack.size() > 0 )
            {
                if( !arrived_at_checkpoint )
                {
                    arrived_at_checkpoint = true;
                    checkpoint_arrival_time = ros::Time::now();
                }

                if( (ros::Time::now() - checkpoint_arrival_time) > ros::Duration(loiter_duration) )
                {
                    std::lock_guard<std::mutex> lock(nav_stack_mx);

                    //by setting last setpoint to where we reached
                    LAST_SETPOINT = NEXT_SETPOINT;

                    //And then popping out the next element
                    drone_msgs::waypoint& next_pose = output_nav_stack.front();
                    output_nav_stack.pop_front();

                    ROS_WARN("Moving to next setpoint [%s] %f %f %f", next_pose.cmd.c_str(), next_pose.pose.position.x, next_pose.pose.position.y, next_pose.pose.position.z);

                    //If we are still going for waypoints
                    if( next_pose.cmd == "w" )
                    {
                        pose_to_transform( NEXT_SETPOINT, next_pose.pose );
                        //Need to always compute distance information or we have no idea how to make progress
                        compute_setpoint_distance();
                        arrived_at_checkpoint = false;
                    }
                    //Otherwise, it should be a landing request
                    else if( next_pose.cmd == "l" )
                    {
                        //We need to ask the drone to land
                        drone->setMode("AUTO_LAND");
                    }
                    // Othewise... something is very malformed
                    else
                    {
                        //Throw an error, but still try to land
                        ROS_ERROR("Unknown comand [%s]: landing the drone!", next_pose.cmd.c_str());
                        drone->setMode("AUTO_LAND");
                    }
                }
            }
        }

        rate.sleep();
    }
}

/**
 * Offboard Publisher: actually sends controls to the drone
 */
void offboardControlPublisher()
{
    // Has to be faster that 2Hz or else connection to OFFBOARD will be dropped. (2Hz can be changed). 
    ros::Rate rate(10.0);
    // This only serves to publish the desired setpoint
    while (ros::ok())
    {
        setpointRawPub.publish( vel_flight_controller );

        rate.sleep();
   }
}

/**
 * Offboard Goal publisher: sends goal points to avoidance
 */
void offboardGoalPublisher()
{
    // Has to be faster that 2Hz or else connection to OFFBOARD will be dropped. (2Hz can be changed). 
    ros::Rate rate(10.0);
    // This only serves to publish the desired setpoint
    while (ros::ok())
    {
        clickedGoalPub.publish( goal_flight_controller );

        rate.sleep();
   }
}

#endif

