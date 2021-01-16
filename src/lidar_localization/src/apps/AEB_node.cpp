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

    //禁区划分
    float xL = -1.2; //车左右1.2米为禁区
    float xR = 1.2;
    float yN = 2.0; //车前2m，到车前7米为禁区
    float yF = 10.3;

    std::shared_ptr<GridMapSubscriber> grid_map_sub_ptr = std::make_shared<GridMapSubscriber>(nh, "/grid_map", 100); //定义一个发布删格地图的话题
    std::shared_ptr<GridMapPublisher> grid_map_pub_ptr = std::make_shared<GridMapPublisher>(nh, "/grid_map_obstacle_modified", 100, "map"); //发布修改后的栅格

    std::shared_ptr<AEB_operator> AEB_operation_ptr = std::make_shared<AEB_operator>(xL, xR, yN, yF);

    ros::Publisher pub_obstacle  = nh.advertise<std_msgs::Int8>("/obstacle_detection", 100);
    std::deque<nav_msgs::OccupancyGrid> grid_map_data_buff; //保存收到的栅格地图数据

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        grid_map_sub_ptr->ParseData(grid_map_data_buff); //将收到的点云数据push到cloud_data_buff的队尾
        while (grid_map_data_buff.size() > 0)            //把cloud里面的所有数据都处理干净
        {
            nav_msgs::OccupancyGrid grid_map_data = grid_map_data_buff.front();
            grid_map_data_buff.pop_front();
            bool hasObstale = false;
            AEB_operation_ptr->LidarAEB(grid_map_data, hasObstale);
            pub_obstacle.publish(AEB_operation_ptr->obstacle_msg_);
            grid_map_pub_ptr->Publish(grid_map_data);

        }
        rate.sleep();
    }
    return 0;
}
//对‘lidar_localization::TrivialMsgPublisher<nav_msgs::OccupancyGrid_<std::allocator<void> > >::TrivialMsgPublisher(ros::NodeHandle&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, unsigned long, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >)’未定义的引用