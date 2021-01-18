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
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

using std::string;
using std::vector;

using namespace Eigen;
using namespace std;

namespace lidar_localization
{
    
    //最新：所有点的搜索和判断都在车辆坐标系下进行，而两个点之间是否有障碍物，用check_collision在栅格地图坐标系下进行判断
    //搜索范围就是 x=[0+0.3,height-0.3]  y=[-width/2+0.3,width/2-0.3] 

    class RRT
    {
    public:
        //100表示检测到实际的栅格点,90表示根据障碍物根据车辆半径膨胀得到的点，80表示可以行驶，70表示规划的时候需要避开,如果70没办法规划出一条路径，则用80规划
        enum{LETHAL_OBSTACLE=100,INSCRIBED_OBSTACLE=90, WALKING_SPACE=80, PLANNING_SPACE=70};


        typedef struct Node
        {
            float x;
            float y;
            struct Node *parent;
            float cost;
            Node(float x_ = 0.0, float y_ = 0.0, Node *parent_ = NULL, float cost_ = 0.0) : x(x_), y(y_), parent(parent_), cost(cost_) {} // 默认为空，parent
           
        } node;

    public:
        RRT(){grid_map_obstacle_value=WALKING_SPACE;};
        void init(const nav_msgs::OccupancyGrid &grid_map);
        ~RRT();

    public:
        nav_msgs::Path GetPath(const RRT::node &startPoint_, const RRT::node &endPoint_);
        nav_msgs::Path GetPlanningPath(const RRT::node &startPoint_, const RRT::node &endPoint_);
        bool check_collision_between_two_grid_point(const RRT::node *node1, const RRT::node *node2);
        bool check_collision_in_two_vector(const vector<RRT::node> &map_point_in_car, const nav_msgs::OccupancyGrid &grid_map);
        bool check_collision_between_two_car_point(const RRT::node *node1, const RRT::node *node2) ;       

        
    private:
        void GetRanNode(RRT::node *randNode);
        //bool CheckCollision(const node &randNode);
        void GetNearestNode(const RRT::node *randNode,const vector<RRT::node*>&  m_nodeList,node *&m_nearestNode,bool& hasFindNearestPoint); 

        void find_neighbor_node_for_newNode(node *newNode, vector<int> &nearNodeIndexIn_m_nodeList,const vector<RRT::node*>&  m_nodeList);
        //在半径为search_radius的圆内找到一个最好的父节点
        void search_best_parent(node *newNode, const vector<int> &nearNodeIndexIn_m_nodeList,const vector<RRT::node*>&  m_nodeList);
        //查看newNode周围的点是否能用newNode作为父节点
        void reWire_For_NewNode_Neighbor(node *newNode, vector<int> &nearNodeIndexIn_m_nodeList, vector<RRT::node*>&  m_nodeList);

        vector<node> bresenham_k_lessthan_1(int x0, int y0, int x1, int y1);
        vector<node> bresenham_check(int x0, int y0, int x1, int y1);
        void node_path_clear();

    private:
        void get_grid_coordinate_from_car(float x, float y, RRT::node *grid_coordinate);      //返回栅格地图的坐标
        void get_grid_coordinate_from_index(int grid_index, RRT::node *grid_coordinate);      //由栅格地图index得到栅格地图坐标
        void get_car_coordinate(const RRT::node *grid_coordinate, RRT::node *car_coordinate); //由栅格地图坐标得到车辆坐标系下坐标
        int get_index_from_grid(const RRT::node &node1);
        int getIndexFromPoseInCarCoordinate(const float x, const float y);
        void expand_map(nav_msgs::OccupancyGrid &map);

    private:
        float m_minRan;
        float m_maxRan;

        float m_size; //每次搜索前进的距离
        float m_step;
        float search_radius; //找寻随机点的最近点的搜索范围，单位是栅格地图坐标

        float rand_x_min;//随即点的搜索范围
        float rand_x_max;
        float rand_y_min;
        float rand_y_max;

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
        
        std::mutex my_mutext;
        int grid_map_obstacle_value;//表示规划时需要避开的栅格地图的障碍物值。这个值在规划的时候首先取PLANNING_SPACE=70进行规划，尽可能的离障碍物远一点
                                    //如果没有路径，则用WALKING_SPACE=80进行规划。但是在用two_vector进行全局路径判断障碍物时仍用WALKING_SPACE=80来进行判断

    public:
        //nav_msgs::Path path_;//将路径保存为path
        nav_msgs::OccupancyGrid map_;
        nav_msgs::OccupancyGrid vis_map_; //为了可视化路径
    };
} // namespace lidar_localization
#endif
