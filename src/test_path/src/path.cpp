#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <gnss_pos/pos_xy.h>
#include<geometry_msgs/Point32.h>
#include<geometry_msgs/PolygonStamped.h>
#include<nav_msgs/OccupancyGrid.h>
#include<nav_msgs/MapMetaData.h>
#include<cstring>

using namespace std;
main (int argc, char **argv)
{
    ros::init (argc, argv, "showpath");

    ros::NodeHandle ph;
    ros::Publisher path_pub = ph.advertise<nav_msgs::Path>("trajectory",1, true);
    ros::Publisher pos_pub = ph.advertise<geometry_msgs::PoseStamped>("position",1, true);
    ros::Publisher poly_pub = ph.advertise<geometry_msgs::PolygonStamped>("rectangle",1,true);
    ros::Publisher map_pub = ph.advertise<nav_msgs::OccupancyGrid>("/my_map",1,true);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nav_msgs::Path path;
    nav_msgs::OccupancyGrid my_map;
    nav_msgs::MapMetaData info;

    my_map.info = info;




    //geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped this_pose_stamped;

    geometry_msgs::Point32 pointPoly;
    geometry_msgs::PolygonStamped rectangle;

    //nav_msgs::Path path;
    string header = "odom1";
    path.header.stamp=current_time;
    this_pose_stamped.header.stamp = current_time;
    path.header.frame_id= header;
    this_pose_stamped.header.frame_id = header;
    rectangle.header.frame_id = header;
    my_map.header.frame_id = header;


    double x = 0.0;
    double y = 0.0;
    double th = 0.0;
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;

    ros::Rate loop_rate(1);



    double x1,y1,z1;
    float data[4][2] = {{0.0f,0.0f},{5.0f,0.0f},{5.0f,10.0f},{0.0f,10.0f},};

    for (int i=0;i<4;i++){
        pointPoly.x = data[i][0];
        pointPoly.y = data[i][1];
        pointPoly.z = 0.0;
        rectangle.polygon.points.push_back(pointPoly);
    }

    while (ros::ok())
    {

        current_time = ros::Time::now();
        //compute odometry in a typical way given the velocities of the robot
        double dt = 0.1;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;


  
        this_pose_stamped.pose.position.x = x;
        this_pose_stamped.pose.position.y = y;

        geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
        this_pose_stamped.pose.orientation.x = goal_quat.x;
        this_pose_stamped.pose.orientation.y = goal_quat.y;
        this_pose_stamped.pose.orientation.z = goal_quat.z;
        this_pose_stamped.pose.orientation.w = goal_quat.w;

        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom1";
        path.poses.push_back(this_pose_stamped);

        path_pub.publish(path);
        pos_pub.publish(this_pose_stamped);
        poly_pub.publish(rectangle);
        

        ros::spinOnce();               // check for incoming messages

        last_time = current_time;
        loop_rate.sleep();
    }

    return 0;
}


