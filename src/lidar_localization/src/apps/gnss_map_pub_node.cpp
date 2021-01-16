/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/GNSS_trajectory/GNSS_trajectory.hpp"
#include "lidar_localization/publisher/path_publisher.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"

using namespace lidar_localization;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "gnss_map_pub_node");
    ros::NodeHandle nh;

    string csv_path =  ("/home/gxf/catkin_ws_boonrayCar/src/lidar_localization/latlon_data/map.csv");
    std::shared_ptr<TrajectoryOperator> gnss_trajectory_ptr = std::make_shared<TrajectoryOperator>(csv_path); //订阅惯导数据
    //ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1, true);
    std::shared_ptr<PathPublisher> path_pub_ptr=std::make_shared<PathPublisher>(nh, "global_path_pub", "/map", "/global_path", 100);
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map_points", "/map", 100);

    std::deque<GNSSData> gnss_data_buff;

    ros::Rate rate(1);
    gnss_trajectory_ptr->ReadCsv();                 //读取数据
    gnss_trajectory_ptr->ParseData(gnss_data_buff); //将数据存入变量
    
    //发布全局数据
    while (ros::ok())
    {
        ros::spinOnce();
        path_pub_ptr->Publish(gnss_data_buff);
        cloud_pub_ptr->Publish_GPS2xy(gnss_data_buff);
        std::cout<<"gnss size= "<<gnss_data_buff.size()<<std::endl;
        rate.sleep();
    }

    return 0;
}