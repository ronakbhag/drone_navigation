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
/* Node to control the drone with position setpoints at a given height */

#include "path_follow_services.h"
#include "ros_util.h"
#include "visualization/rviz_util.h"


/**
 * Path follower node executable
 */
int main(int argc, char**argv)
{
    ros::init(argc, argv, "path_follower_node");
    ros::NodeHandle nh;

    drone = new drone_interface::Drone();

    visualization_marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

    //==== Need to set up our basicparameters ===//
    LOOK_AHEAD = GetParam<double>(nh, ros::this_node::getName() + "/look_ahead", 0.88);
    DISTANCE_THRESHOLD = GetParam<double>(nh, ros::this_node::getName() + "/distance_threshold", 0.22);
    PATH_FOLLOW_RATE = GetParam<double>(nh, ros::this_node::getName() + "/follower_rate", 20.0);

    use_avoidance = GetParam<bool>(nh, ros::this_node::getName() + "/use_avoidance", false);
    use_vision = GetParam<bool>(nh, ros::this_node::getName() + "/use_vision", true);
    self_start = GetParam<bool>(nh, ros::this_node::getName() + "/self_start", true);
    autonomous_control = GetParam<bool>(nh, ros::this_node::getName() + "/autonomous_control", false);

    initial_wait_duration = GetParam<double>(nh, ros::this_node::getName() + "/initialization_duration", 3.0);
    loiter_duration = GetParam<double>(nh, ros::this_node::getName() + "/loiter_duration", 0.0);

    // Use setpoint raw plugin in MAVROS for giving position setpoints at a given height
    ros::ServiceServer setpoint_service = nh.advertiseService("/airlitix/set_position", go_to_point);
    ros::ServiceServer path_follow_service = nh.advertiseService("/airlitix/follow_path", follow_path);

    if( use_avoidance )
    {
        clickedGoalPub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    }
    else
    {
        setpointRawPub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    }

    vel_flight_controller.type_mask =
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;
    //TODO: excuse me, but why NED?
    vel_flight_controller.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // Given height to fly at
    goal_flight_controller.pose.position.x = vel_flight_controller.position.x = 0.00001;
    goal_flight_controller.pose.position.y = vel_flight_controller.position.y = 0.00001;
    goal_flight_controller.pose.position.z = vel_flight_controller.position.z = 1.0;
    goal_flight_controller.pose.orientation.x = 0.0;
    goal_flight_controller.pose.orientation.y = 0.0;
    goal_flight_controller.pose.orientation.z = 0.0;
    goal_flight_controller.pose.orientation.w = 1.0;
    vel_flight_controller.yaw = vel_flight_controller.yaw_rate = 0.0;

    // Start a new thread so that commands are periodically sent to the controller or else
    // the mode will change from offboard some other mode.
    // Points have to be ready before thread starts.
    auto pubFun = ( use_avoidance ? offboardGoalPublisher : offboardControlPublisher );
    std::thread offboardThread( pubFun );
    ROS_INFO("OFFBOARD Control Thread Started");

    while( ros::ok() )
    {
        // We're going to start the path following command, and then wait for the drone to be armed
        std::thread distThread( pathFollowCallback );
        ROS_INFO("[Path Follower] Path-following Command Thread Started");

        // Set the proper initialization flags
        std::vector<std::string> init_flags = {};
        if( self_start )
            init_flags.push_back("arm");
        if( use_vision )
            init_flags.push_back("vision");
        else
            init_flags.push_back("odom");
        if( autonomous_control )
            init_flags.push_back("autonomous");
        drone->setInitFlags( init_flags );

        // Do the actual initialization
        if( !drone->Initialize() )
        {
            ROS_ERROR("[Path Follower] The Drone did not initialize appropriately.");
            return -1;
        }

        //If ROS has crapped out on us at this point for some reason, we need to bail out.
        if( !ros::ok() )
        {
            ROS_ERROR("[Path Follower] can't see ROS, exiting...");
            return -1;
        }

        // After everything is initialized, we might have to wait for a human
        ros::Rate rate(20);
        while( ros::ok() && drone->getCurrentState().mode != "OFFBOARD" )
        {
            ros::spinOnce();
            rate.sleep();
        }

        // Now that autonomous control is ready, start actually updating target
        while( ros::ok() && drone->ready() )
        {
            ros::spinOnce();

            // If using avoidance, send a "clicked point"
            if( use_avoidance )
            {
                goal_flight_controller.pose.position.x = TARGET.getOrigin()[0];
                goal_flight_controller.pose.position.y = TARGET.getOrigin()[1];
                goal_flight_controller.pose.position.z = TARGET.getOrigin()[2];
            }
            else
            {
                //TODO: shouldn't we use orientation information?
                vel_flight_controller.position.x = TARGET.getOrigin()[0];
                vel_flight_controller.position.y = TARGET.getOrigin()[1];
                vel_flight_controller.position.z = TARGET.getOrigin()[2];
            }

            visualization_marker_pub.publish( pmarker( "cube", "navigation", 3000, {0.015,0.015,0.015}, {NEXT_SETPOINT.getOrigin()[0],NEXT_SETPOINT.getOrigin()[1],NEXT_SETPOINT.getOrigin()[2]}, {0,0,0,1}, {0.0,1.0,0.0,0.6}, "map" ) );
            visualization_marker_pub.publish( pmarker( "cube", "navigation", 3001, {0.015,0.015,0.015}, {LAST_SETPOINT.getOrigin()[0],LAST_SETPOINT.getOrigin()[1],LAST_SETPOINT.getOrigin()[2]}, {0,0,0,1}, {0.4,0.4,0.6,0.5}, "map" ) );
            visualization_marker_pub.publish( pmarker( "cube", "navigation", 3002, {0.013,0.013,0.013}, {TARGET.getOrigin()[0],TARGET.getOrigin()[1],TARGET.getOrigin()[2]}, {0,0,0,1}, {0.9,0.7,0.0,0.4}, "odom" ) );

            rate.sleep();
        }

        ROS_WARN("[Path Follower] Either we are done, or something went wrong with the drone!");

        // If something bad has happened, let's just stop the dist thread
        distThread.join();
    }
    //I feel like this should be stopped too, but we can't stay in OFFBOARD mode unless this is running?
    offboardThread.join();

    delete drone;

    return 0;
}
