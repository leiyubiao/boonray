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

//tf
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

//添加client
#include <boost/shared_ptr.hpp>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8.h>
#include <Eigen/Dense>
using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{
    //坐标系 前x右y,
    //输入点云，可以加一个激光雷达转换矩阵，并且输入建立删格地图，并发布出去，
    class AEB_operator
    {
    public:
        AEB_operator(float carFrontNear,float carFrontFar,float carLeft,float carRight):xN_(carFrontNear),xF_(carFrontFar),yL_(carLeft),yR_(carRight){}
        void LidarAEB( nav_msgs::OccupancyGrid &map, bool &hasObstacle); //根据输入的地图判断是否
        void LidarAEB_AckermanModel(nav_msgs::OccupancyGrid &map, bool &hasObstacle,float speed,float delta,float Wheelbase);
        std_msgs::Int8 obstacle_msg_;
        
    private:
        float xN_ = 1.0; //车前1-8米为禁区
        float xF_ = 8.0;
        float yL_ = 0.5; //车左0.5米
        float yR_ = -0.5;
        
    };
} // namespace lidar_localization
#endif
