/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include "glog/logging.h"
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/publisher/obstacle_warn_publisher.hpp"
#include "lidar_localization/publisher/grid_map_publisher.hpp"
#include "lidar_localization/subscriber/grid_map_subscriber.hpp"
#include "lidar_localization/obstacle_operation/AEB.hpp"
#include "lidar_localization/publisher/path_publisher.hpp"
#include "lidar_localization/path_planning/RRT.hpp"

#include <iostream>
using namespace lidar_localization;
using namespace std;
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "AEB_node");
    ros::NodeHandle nh;

    std::shared_ptr<GridMapSubscriber> grid_map_sub_ptr = std::make_shared<GridMapSubscriber>(nh, "/grid_map", 100); //定义一个发布删格地图的话题
    std::shared_ptr<RRT> RRT_ptr = std::make_shared<RRT>();
    std::deque<nav_msgs::OccupancyGrid> grid_map_data_buff;                                                                         //保存收到的栅格地图数据
    std::shared_ptr<GridMapPublisher> grid_map_pub_ptr = std::make_shared<GridMapPublisher>(nh, "/RRT_debug_grid_map", 100, "map"); //定义一个发布删格地图的话题

    ros::Rate rate(1);

    //RRT::node *node1 = new RRT::node(0, 0);
    //mntRRT::node *node2 = new RRT::node(4, -5);
    std::shared_ptr<PathPublisher> RRT_path_pub_ptr = std::make_shared<PathPublisher>(nh, "RRT_path_pub", "/map", "/RRT_path", 100); //!!因为里面vector没办法清空，所以只能每次重新定义变量
    while (ros::ok())
    {
        ros::spinOnce();

        grid_map_sub_ptr->ParseData(grid_map_data_buff); //将收到的点云数据push到cloud_data_buff的队尾
        ros::Time time_begin=ros::Time::now();
        //每次只处理队尾的点
        if (grid_map_data_buff.size() > 0) //把cloud里面的所有数据都处理干净
        {
            nav_msgs::OccupancyGrid grid_map_data = grid_map_data_buff.front();

            grid_map_data_buff.erase(grid_map_data_buff.begin(), grid_map_data_buff.end());

            /**********RRT***********/
            RRT::node start_p = RRT::node(5, 0);
            RRT::node end_p = RRT::node(10, 0); //实际车辆坐标系下的坐标系

            RRT_ptr->init(grid_map_data);
            nav_msgs::Path RRT_path = RRT_ptr->GetPath(start_p, end_p);
            /*************************/
            RRT_path_pub_ptr->PublishNavPath(RRT_path);
            grid_map_pub_ptr->Publish(RRT_ptr->vis_map_);
        }
        ros::Time time_end=ros::Time::now();
        double ros_duration=(time_end-time_begin).toSec();
        cout<<" RRT time ="<<ros_duration<<endl;
        rate.sleep();
    }
    return 0;
}
