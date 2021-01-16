#ifndef GRID_MAP_HPP_
#define GRID_MAP_HPP_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <cmath>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>

////PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

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


using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{
    
    struct JudgeNode
    {
        int x = 0;
        int y = 0;
        int index;
        double max = 0.0;
        double min = 0.0;
        bool visited = false;
    };

    //输入点云，可以加一个激光雷达转换矩阵，并且输入建立删格地图，并发布出去，
    class GridMapConstructor
    {
    public:
        GridMapConstructor();
        void filterVoxelGrid(const sensor_msgs::PointCloud2ConstPtr &Inpointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr Outpointcloud, double th);
        void ConstructMapHeightDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map); //高程差建图
        void ConstructMapABSHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map);  //绝对高度建图

    private:
        int Map_width;
        int Map_height;
        int kernal_di;
        int kernal_er;
        double width;
        double height;
        double resolution;
        double offsetx;
        double offsety;
        double limit_height;
        double limit_low;
        double height_diff;    //高程差阈值
        double voxelthreshold; //体素滤波的阈值
        double HeightUpper;    //这是在用绝对高度的时候用
        double HeightLower;
        Matrix3d RotationMatrix;
    };
} // namespace lidar_localization
#endif
