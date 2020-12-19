#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <deque>
#include "drone_msgs/commandPath.h"
#include "drone_msgs/waypoint.h"



double WP_THRESHOLD = 0.3; //threshold distance from waypoint to consider waypoint reached
mavros_msgs::State current_state;
ros::Publisher local_pos_pub;
geometry_msgs::Point local_pos; //current local position of the drone
geometry_msgs::Quaternion local_orientation;

std::deque<drone_msgs::waypoint> wp_queue;

drone_msgs::waypoint current_goal_wp;


// currently i can only think of these two states, things like emergency etc can be added later
enum PlannerState {STANDBY, PATH_FOLLOW};
// initialize in standby state
PlannerState ps = STANDBY;

// call back for mavros state subscriber
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

// callback for drone position in local frame
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = msg->pose.position;
    local_orientation = msg->pose.orientation;
}
//service callback for follow_path service client
bool follow_path( drone_msgs::commandPath::Request  &req,
                  drone_msgs::commandPath::Response &res)
{

    for( unsigned i=1; i<req.waypoints.size(); ++i )
    {
        wp_queue.push_back( req.waypoints[i] );
    }
    current_goal_wp = wp_queue.front();
    wp_queue.pop_front();
    ps = PATH_FOLLOW;
    res.accepted = true;
    return true;
}

// calculates eucledian dist b/w current position and goal, 
// if it is less than a threshold, returns true
bool check_waypoint_reached(drone_msgs::waypoint wp)
{
    auto pos = wp.pose.position;
    double dist = sqrt(pow(local_pos.x-pos.x,2) + pow(local_pos.y-pos.y,2) + pow(local_pos.z-pos.z,2)) ;
    if (dist< WP_THRESHOLD)
        return true;
    else 
        return false;
}

void publish_local_pose(drone_msgs::waypoint wp)
{
    geometry_msgs::PoseStamped pose_to_pub;
    pose_to_pub.pose = wp.pose;
    pose_to_pub.pose.orientation = local_orientation;   
    pose_to_pub.header.stamp = ros::Time::now();
    local_pos_pub.publish(pose_to_pub);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_control_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceServer path_follow_service = nh.advertiseService("/airlitix/follow_path", follow_path);

    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "POS_CTRL: Waiting for FCU connection");
        rate.sleep();
    }


    while(ros::ok())
    {
        if( (current_state.mode != "OFFBOARD") || (!current_state.armed)) 
        {
            ROS_INFO_THROTTLE(1, "POS_CTRL: Waiting for arming");
        }

        if(ps==STANDBY)
        {
            ROS_DEBUG_THROTTLE(1, "POS_CTRL: Waiting for path");
            //do nothing
        }
        else if(ps == PATH_FOLLOW)
        {
            if (wp_queue.size() == 0)
            {
                // give landing command if the last wp cmd is "l"
                if(current_goal_wp.cmd == "l")
                {
                    mavros_msgs::SetMode set_mode_req;
                    set_mode_req.request.custom_mode = "AUTO_RTL";
                    ros::service::call("mavros/set_mode", set_mode_req);
                    ROS_INFO("POS_CTRL: Landing");
                    ps = STANDBY;
                }
                // if no additional waypoints, and waypoint also reached
                else if (check_waypoint_reached(current_goal_wp))
                {
                    // path is complete, fall back to standby mode
                    ROS_INFO("POS_CTRL: Path complete");
                    ps = STANDBY;
                }
                else 
                {
                    publish_local_pose(current_goal_wp);//
                }
            }
            else
            {
                //@TODO add check for landing 
                //if intemediary wp achieved, change goal to next waypoint
                if (check_waypoint_reached(current_goal_wp))
                {
                    ROS_INFO("POS_CTRL: Executing next waypoint");
                    current_goal_wp = wp_queue.front();
                    wp_queue.pop_front();
                }
                publish_local_pose(current_goal_wp);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}