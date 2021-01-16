#ifndef RRT_HPP_
#define RRT_HPP_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <cmath>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <math.h>
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
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include<lidar_localization/path_tracking/purePursuit.hpp>

using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{
    //输入的是栅格地图，相对于车辆坐标系（前x右y上z）的起点（一般为0,0），终点坐标系，搜索范围为整个栅格，也可以缩小一点
    //里面所有的搜索都是在栅格地图坐标系里面进行的,node还是用浮点型，因为这样方便计算距离，不过在作为index的时候需要强制转换
    class RRT
    {
    public:
        typedef struct Node
        {
            float x;
            float y;
            struct Node *parent;
            float cost;
            Node(float x_ = 0.0, float y_ = 0.0, Node *parent_ = NULL, float cost_ = 0.0) : x(x_), y(y_), parent(parent_), cost(cost_) {} // 默认为空，parent

        } node;

    public:
        RRT(){};
        void init(const node &startPoint_, const node &endPoint_, const nav_msgs::OccupancyGrid &grid_map);
        ~RRT();


    /*****************************************此处为雷玉标新加函数****************************************************/

    public:
        bool m_flag_for_RRT; // true 说明要进行RRT* 路径规划。默认为false.
        bool m_flag_for_endRRT; // RRT 结束的标志，true  代表结束了。

        //purePursuit::Position m_goal; // RRT* 的终点
       // purePursuit::ctrlCommand m_vehicle_cmd;
        Node m_goal_node; 
        //vector<purePursuit::Position> wayPoints; // 在构造函数中进行初始化

        //purePursuit* my_pure_pursuit;
    /**************************************************************************************************************/

    public:
        nav_msgs::Path GetPath();
        bool check_collision_between_two_grid_point(const RRT::node *node1, const RRT::node *node2);
        bool check_collision_in_two_vector(const vector<RRT::node> &map_point_in_car,const nav_msgs::OccupancyGrid &grid_map);

        node *m_startPoint; //保存路径的起点
        node *m_endPoint;   //保存路径的终点
    private:
        void GetRanNode(RRT::node *randNode);
        //bool CheckCollision(const node &randNode);
        void GetNearestNode(const RRT::node *randNode); //void Visualize();

        void find_neighbor_node_for_newNode(node *newNode, vector<int> &nearNodeIndexIn_m_nodeList);
        //在半径为search_radius的圆内找到一个最好的父节点
        void search_best_parent(node *newNode, const vector<int> &nearNodeIndexIn_m_nodeList);
        //查看newNode周围的点是否能用newNode作为父节点
        void reWire_For_NewNode_Neighbor(node *newNode, vector<int> &nearNodeIndexIn_m_nodeList);

        vector<node> bresenham_k_lessthan_1(int x0, int y0, int x1, int y1);
        vector<node> bresenham_check(int x0, int y0, int x1, int y1);
        void node_path_clear();

    private:
        void get_grid_coordinate_from_world(float x, float y, RRT::node *grid_coordinate);        //返回栅格地图的坐标
        void get_grid_coordinate_from_index(int grid_index, RRT::node *grid_coordinate);          //由栅格地图index得到栅格地图坐标
        void get_world_coordinate(const RRT::node *grid_coordinate, RRT::node *world_coordinate); //由栅格地图坐标得到车辆坐标系下坐标
        int get_index_from_grid(const RRT::node &node1);
        int getIndexFromPoseInCarCoordinate(float x, float y);
        void expand_map(nav_msgs::OccupancyGrid &map);

    private:
        vector<node *> m_nodeList; //保存已经搜寻到的点
        vector<node *> m_path;

        node *m_nearestNode; //保存最近点
        node *nodeRandom;    //定义一个随机点变量来保存随机点

    private:
        float m_minRan;
        float m_maxRan;
        float m_size; //每次搜索前进的距离
        float m_step;
        float search_radius; //找寻随机点的最近点的搜索范围，单位是栅格地图坐标

        float grid_origin_x; //grid_map
        float grid_origin_y;
        float grid_reso;
        int grid_height;
        int grid_width;

        float car_width;  //车长
        float car_length; //车宽

        int max_index; //保存最大的栅格地图的编号
        std::mutex my_mutex;
        int max_iter_time; //最大搜索步数
    public:
        //nav_msgs::Path path_;//将路径保存为path
        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid vis_map_; //为了可视化路径
    };
} // namespace lidar_localization
#endif
