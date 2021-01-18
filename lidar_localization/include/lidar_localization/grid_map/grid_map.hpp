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

//PCL
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
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>

using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{

    //输入点云，可以加一个激光雷达转换矩阵，并且输入建立删格地图，并发布出去，
    class GridMapConstructor
    {
    public:
        enum{LETHAL_OBSTACLE=100,INSCRIBED_OBSTACLE=90, WALKING_SPACE=80, PLANNING_SPACE=70,FREE_SPACE=0};
        struct JudgeNode
        {
            int x;
            int y;
            int index;
            double min_z = 0.0;//当前删格最小的z值
            int higherHeightDiffThanMinZNumber;//比最小的z值大HeightDiff的点数
            JudgeNode(int x_=0,int y_=0, int index_ = 0, double min_z_ = 9999,int higherHeightDiffThanMinZNumber_=0) :x(x_),y(y_), index(index_), min_z(min_z_),higherHeightDiffThanMinZNumber(higherHeightDiffThanMinZNumber_) {}
        };
        
        GridMapConstructor();
        void filterVoxelGrid(const sensor_msgs::PointCloud2ConstPtr &Inpointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr Outpointcloud, double th);
        void ConstructMapHeightDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map); //高程差建图
        void ConstructMapABSHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map);  //绝对高度建图
        void expand_map(nav_msgs::OccupancyGrid &map);
        void get_grid_coordinate_from_car(const float x, const float y, int &index, bool &HasCrossedTheGridMap); //返回栅格地图的坐标,如果点的索引超过
        bool check_collision_between_two_grid_point(const int index1, const int index2);                         //
        vector<JudgeNode> bresenham_k_lessthan_1(int x0, int y0, int x1, int y1);
        vector<JudgeNode> bresenham_check(int x0, int y0, int x1, int y1);

        //有碰撞，则返回true,经过测试，
        bool check_collision_between_two_grid_point(float x1_InCarCoordinate, float y1_InCarCoordinate, float x2_InCarCoordinate, float y2_InCarCoordinate);
        void get_grid_coordinate_from_car(float x, float y, JudgeNode &grid_coordinate); //返回栅格地图的坐标
        //栅格地图索引得到栅格地图坐标
        bool get_grid_coordinate_from_index(int grid_index, JudgeNode *grid_coordinate);          //由栅格地图index得到栅格地图坐标
        void get_world_coordinate(const JudgeNode *grid_coordinate, JudgeNode *world_coordinate); //由栅格地图坐标得到车辆坐标系下坐标
        int get_index_from_grid(const JudgeNode &node, bool &HasCrossedGridMap);

    private:
        int kernal_di;
        int kernal_er;

        double width;
        double height;
        int grid_height;
        int grid_width;
        double grid_origin_x;
        double grid_origin_y;
        int max_index;
        int min_index;
        double grid_reso;

        double limit_height;
        double limit_low;
        double height_diff;    //高程差阈值
        double voxelthreshold; //体素滤波的阈值
        double HeightUpper;    //这是在用绝对高度的时候用
        double HeightLower;
        double carHeight; //标识z轴上面多高的地方不再检测障碍物
        double car_width;
        int planning_radius;
        Matrix3d RotationMatrix;
        nav_msgs::OccupancyGrid map_;
    };
} // namespace lidar_localization
#endif
