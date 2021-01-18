#ifndef GNSS_TRAJECTORY_HPP_
#define GNSS_TRAJECTORY_HPP_

#include "ros/ros.h"
#include <cmath>
#include <iostream>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <mutex>

#include "Geocentric/LocalCartesian.hpp"
#include "lidar_localization/sensor_data/gnss_data.hpp"
#include <deque>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <math.h>
#include <iomanip>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;
namespace lidar_localization
{
    //输入点云，可以加一个激光雷达转换矩阵，并且输入建立删格地图，并发布出去，
    class TrajectoryOperator
    {
    public:
        TrajectoryOperator(string csv_path);                 //输入的csv文件路径
        void SaveCsv( std::deque<GNSSData> &waypoints); //保存未CSV文件
        void ReadCsv();                                      //读取CSV文件，并保存到vector里面
        bool Discrete(const GNSSData &pre, const GNSSData &cur);
        void ParseData(std::deque<GNSSData>& deque_GNSS_data);
        nav_msgs::Path GeneratePath();
        std::deque<GNSSData> gnss_data_buff_; //保存读取的数据
        
        string csv_path_;
    private:
    std::mutex buff_mutex_;
    

    };
} // namespace lidar_localization
#endif