#ifndef AEB_HPP_
#define AEB_MAP_HPP_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <cmath>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>

//PCL

//tf
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

//添加client
#include <boost/shared_ptr.hpp>
//#include <costmap_2d/costmap_2d_ros.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>

using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{
    //坐标系 前x右y
    //输入点云，可以加一个激光雷达转换矩阵，并且输入建立删格地图，并发布出去，
    class AEB_operator
    {
    public:
        AEB_operator(float xN,float xF,float yL,float yR):xN_(xN),xF_(xF),yL_(yL),yR_(yR){}
        void LidarAEB( nav_msgs::OccupancyGrid &map, bool &hasObstacle); //根据输入的地图判断是否
        std_msgs::Int8 obstacle_msg_;
    private:
        float xN_ = 1.0; //车左右1.2米为禁区
        float xF_ = 8.0;
        float yL_ = 0.5; //车前2m，到车前7米为禁区
        float yR_ = -0.5;
        
    };
} // namespace lidar_localization
#endif
