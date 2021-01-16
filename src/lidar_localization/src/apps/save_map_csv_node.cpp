/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include "lidar_localization/GNSS_trajectory/GNSS_trajectory.hpp"
#include "lidar_localization/tf_listener/tf_listener.hpp"
using namespace lidar_localization;

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "map_save_node");
    ros::NodeHandle nh;
    string csv_path = "/home/boonray/catkin_ws/src/lidar_localization/latlon_data/map.csv";
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/huace_msg", 1000000); //订阅惯导数据
    std::shared_ptr<TrajectoryOperator> gnss_trajectory_ptr = std::make_shared<TrajectoryOperator>(csv_path);

    std::deque<GNSSData> gnss_data_buff;

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    gnss_sub_ptr->ParseData(gnss_data_buff);
    gnss_trajectory_ptr->SaveCsv(gnss_data_buff);

    return 0;
}