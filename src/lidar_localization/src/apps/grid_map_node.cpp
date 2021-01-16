/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include "glog/logging.h"
#include <pcl/common/transforms.h>
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/publisher/grid_map_publisher.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/subscriber/cloud_subscriber.hpp"
#include "lidar_localization/grid_map/grid_map.hpp"
#include <iostream>
using namespace lidar_localization;
using namespace std;
int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "grid_map_node");
    ros::NodeHandle nh;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/pandar", 100000);
    std::shared_ptr<GridMapPublisher> grid_map_pub_ptr = std::make_shared<GridMapPublisher>(nh, "/grid_map", 100, "map"); //定义一个发布删格地图的话题
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_cloud", "map", 100);    //发布激光雷达数据

    std::shared_ptr<GridMapConstructor> grid_map_constructor_ptr = std::make_shared<GridMapConstructor>();

    std::deque<CloudData> cloud_data_buff;

    //x: 0.483m, y: 0.381m, z: 0.3754m, yaw:0.785, roll:0, pitch:0
    //x: 0.483m, y: -0.381m, z: 0.3754m, yaw:0.5.498, roll:0, pitch:0

    Eigen::Matrix4d lidar1_to_car = Eigen::Matrix4d::Identity(4, 4);
    Eigen::AngleAxisd v1(M_PI/4,Eigen::Vector3d(0,0,59*M_PI/180.0));//沿z轴旋转了45度
    lidar1_to_car(0, 3) = 0.483; // 平移
    lidar1_to_car(1, 3) = 0.381;
    lidar1_to_car(2, 3) = 0.3754;
    lidar1_to_car.block<3, 3>(0, 0) = v1.toRotationMatrix(); //利用imu得到的四元数旋转信息得到旋转矩阵
    

    Eigen::Matrix4d lidar2_to_car = Eigen::Matrix4d::Identity(4, 4);
    Eigen::AngleAxisd v2(M_PI/4,Eigen::Vector3d(0,0,0.5498));//沿z轴旋转了45度


    lidar2_to_car(0, 3) = 0.483; // 平移
    lidar2_to_car(1, 3) = -0.381;
    lidar2_to_car(2, 3) = 0.3754;

    lidar2_to_car.block<3, 3>(0, 0) = v2.toRotationMatrix(); //利用imu得到的四元数旋转信息得到旋转矩阵

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff); //将收到的点云数据push到cloud_data_buff的队尾
        while (cloud_data_buff.size() > 0)         //把cloud里面的所有数据都处理干净
        {
            CloudData cloud_data = cloud_data_buff.front();
            cloud_data_buff.pop_front();
            //cout<<"cloud size= "<<cloud_data.cloud_ptr->points.size()<<endl;

            pcl::transformPointCloud(*cloud_data.cloud_ptr, *cloud_data.cloud_ptr, lidar1_to_car); //点云旋转

            nav_msgs::OccupancyGrid grid_map;
            grid_map_constructor_ptr->ConstructMapHeightDiff((cloud_data.cloud_ptr), grid_map);
            grid_map_pub_ptr->Publish(grid_map);
            cloud_pub_ptr->Publish(cloud_data.cloud_ptr);
        }
        rate.sleep();
    }
    return 0;
}