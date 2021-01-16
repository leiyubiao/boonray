/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include "glog/logging.h"
#include <ros/ros.h>

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/publisher/obstacle_warn_publisher.hpp"
#include "lidar_localization/publisher/grid_map_publisher.hpp"
#include "lidar_localization/subscriber/grid_map_subscriber.hpp"
#include "lidar_localization/obstacle_operation/AEB.hpp"
#include "lidar_localization/publisher/path_publisher.hpp"
#include "lidar_localization/path_planning/RRT.hpp"
#include "lidar_localization/GNSS_trajectory/GNSS_trajectory.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include <iostream>
#include<geometry_msgs/PoseStamped.h>
#include<vector>


#include "lidar_localization/path_tracking/purePursuit.hpp"
using namespace lidar_localization;
using namespace std;


// 转成全局路径的路径点。
vector<purePursuit::Position> wayPoints; // 全程路径点。
purePursuit::Position vehiclePos; // 车辆当前实际位置。

purePursuit::ctrlCommand cmd;



int flag_RRT_cmd = 0; // 0代表不执行RRT*的速度与转角指令。1代表要执行。
int flag_path = 0; //    确保用于重规划的终点不变。


nav_msgs::Path RRT_path;
nav_msgs::Path path_in_world;
bool flag_collision = false; // true 说明会发生碰撞，启动RRT*，false  说明没有发生碰撞，不启动RRT*
bool flag_RRT_end = false; // false  代表没有到达终点，true  代表到达了终点，要在 flag_collision 为true的情况下。
bool flag_replanning = false;
int foundIndex; // 预瞄点的索引。
int nearestIndex; // 最近点的索引。

RRT::node end_p; //实际车辆坐标系下的坐标系

purePursuit::PID_variables setPID;





void global_to_vehicle(vector<RRT::node>& node_list,const purePursuit::Position &vehiclePos)
{
    RRT::node tmpNode;
    //int size = node_list.size();
    float dx,dy;
    float tmp_x,tmp_y;
    float theta = vehiclePos.heading * 3.1415926 / 180.0;
    for (auto &p:node_list)
    {
        dx = p.x-vehiclePos.x;
        dy = p.y - vehiclePos.y;
        tmp_x = cos(theta) * dx - sin(theta) * dy;
        tmp_y = sin(theta) * dx + cos(theta) * dy;
        p.x = tmp_x;
        p.y = tmp_y;
    }
}

nav_msgs::Path  vehicle_to_global(const nav_msgs::Path& path_in_vehicle,const purePursuit::Position &vehiclePos)
{
    nav_msgs::Path path_in_world;
    int size = path_in_vehicle.poses.size();
    geometry_msgs::PoseStamped poseTmp;


    float tmp_x,tmp_y;
    float x,y;
    float theta = vehiclePos.heading * 3.1415926 / 180.0;
    for (int i=0; i<size;i++)
    {
        float path_x,path_y;
        path_x = path_in_vehicle.poses[i].pose.position.x;
        path_y = path_in_vehicle.poses[i].pose.position.y;
        tmp_x = cos(theta) * path_x + sin(theta) * path_y;
        tmp_y = -sin(theta) * path_x+ cos(theta) * path_y;
        x = tmp_x + vehiclePos.x;
        y = tmp_y + vehiclePos.y;
        poseTmp.pose.position.x = x;
        poseTmp.pose.position.x = y;
        poseTmp.pose.position.z = 0.0f;
        path_in_world.poses.push_back(poseTmp);

    }
    return path_in_world;
}

RRT::node global_to_vehicle_point(const RRT::node& end_point, const purePursuit::Position &vehiclePos)
{
    RRT::node tmpNode;
    float theta = vehiclePos.heading * 3.1415926 / 180.0;
    float tmp_x,tmp_y;
    float dx,dy;

    dx = end_point.x-vehiclePos.x;
    dy = end_point.y - vehiclePos.y;
    tmp_x = cos(theta) * dx - sin(theta) * dy;
    tmp_y = sin(theta) * dx + cos(theta) * dy;
    tmpNode.x = tmp_x;
    tmpNode.y = tmp_y;

    return tmpNode;
}


purePursuit::ctrlCommand Calculate(const nav_msgs::Path& path_in_world, const purePursuit::Position& vehiclePos)
{
    purePursuit::ctrlCommand cmd;
    int size = path_in_world.poses.size();
    geometry_msgs::PoseStamped poseTmp;
    if (size == 0)
    {
        cmd.steer = 0.0f;
        cmd.speed = 0.0f;
        cmd.brake = 0.0f;
        ROS_ERROR("no path found!!!! stop!!!!!!!");
        return cmd;
    }

    vector<purePursuit::Position> path;
    purePursuit::Position tmp1;
    purePursuit::Position tmp2;
    float tmp_x,tmp_y;
    float xFirst,yFirst;
    float xSecond,ySecond;
    
    float len = 0.2;
    
    for (int i=0; i<size-1;i++){
        float dist = 999999999999.9;
        xFirst = path_in_world.poses[i].pose.position.x;
        yFirst =  path_in_world.poses[i].pose.position.y;
        xSecond = path_in_world.poses[i+1].pose.position.x;
        ySecond =  path_in_world.poses[i+1].pose.position.y;
        float dx = xSecond - xFirst;
        float dy = ySecond - yFirst;
        tmp1.x = xFirst;
        tmp1.y = yFirst;
        tmp2.x = xSecond;
        tmp2.y = ySecond;


        path.push_back(tmp1);
        //path.push_back(tmp2);
        while (dist >= len)
        {
            float theta = atan2(dy,dx);
            tmp_x = xFirst + len*cos(theta);
            tmp_y = yFirst + len*sin(theta);

            path.push_back(tmp1);
            dist = sqrt((tmp_x - xSecond)*(tmp_x - xSecond) + (tmp_y - ySecond)*(tmp_y - ySecond) );
        }

    }

    RRT::node choosen_point;
    float distance;
    float min_value = 999999999999999.9;
    int min_index = 0; 
     int need_index;
    for (unsigned int j=0; j<path.size();j++){
        distance = sqrt( (path[j].x-vehiclePos.x)*(path[j].x-vehiclePos.x) + (path[j].y-vehiclePos.y)*(path[j].y-vehiclePos.y) );
        if (distance < min_value)
        {
            min_value = distance;
            min_index = j;
        }
    }

    need_index = min_index + 25;
    int path_size;
    path_size = path.size();
    if (path_size==0) ROS_ERROR("NO DATA, STOP!!!!!");
    need_index = min(path_size-1, max(0,need_index));
    choosen_point.x = path[need_index].x;
    choosen_point.y = path[need_index].y;

    RRT::node need_point_in_vehicle;
    need_point_in_vehicle = global_to_vehicle_point(choosen_point, vehiclePos);


    float steerCmd;
	float ld;
	float yValue;
	ld = sqrt(need_point_in_vehicle.x * need_point_in_vehicle.x + need_point_in_vehicle.y * need_point_in_vehicle.y);
		
	yValue = -need_point_in_vehicle.y;
	steerCmd = atan(2 * 1.1* yValue / (ld * ld)) * 180.0 / 3.1415926; // if it is wrong, please add "-" before atan();
	steerCmd = min(25.0f, max(-25.0f,steerCmd));
	cmd.steer = steerCmd;
    cmd.speed = 2.0f;
    cmd.brake = 0.0f;
    return cmd;


}

int main(int argc, char *argv[])
{

    setPID.Kp_v = 1.5;
    setPID.Ki_v = 0.0;
    setPID.Kd_v = 0.5;
    setPID.Kp_brake = -30.0;
    setPID.Ki_brake = -0.0;
    setPID.Kd_brake = -6.0;
    purePursuit my_pure_pursuit(wayPoints,1.1, 75, setPID); // 往前找15米，参数可调。
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "AEB_node");
    ros::NodeHandle nh;

    string csv_path = "/home/gxf/catkin_ws_boonrayCar/src/lidar_localization/latlon_data/map.csv";
    std::shared_ptr<TrajectoryOperator> gnss_trajectory_ptr = std::make_shared<TrajectoryOperator>(csv_path);   //订阅惯导数据
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/huace_msg", 1000000); //订阅惯导数据

    std::shared_ptr<GridMapSubscriber> grid_map_sub_ptr = std::make_shared<GridMapSubscriber>(nh, "/grid_map", 100); //定义一个发布删格地图的话题
    std::shared_ptr<RRT> RRT_ptr = std::make_shared<RRT>();
    std::deque<nav_msgs::OccupancyGrid> grid_map_data_buff;                                                                         //保存收到的栅格地图数据
    std::shared_ptr<GridMapPublisher> grid_map_pub_ptr = std::make_shared<GridMapPublisher>(nh, "/RRT_debug_grid_map", 100, "map"); //定义一个发布删格地图的话题

    /******************************读取全局路径*******************************************/
    
    std::deque<GNSSData> csv_gnss_data_buff;       //读取csv里的寻迹路径
    std::deque<GNSSData> new_gnss_data_buff;       //实时收到的惯导消息
   
    gnss_trajectory_ptr->ReadCsv();                     //读取数据
    gnss_trajectory_ptr->ParseData(csv_gnss_data_buff); //将数据存入变量


    for (auto gnss_data : csv_gnss_data_buff)
    {
        if (!gnss_data.origin_position_inited) //以最开始受收到的经纬度为原点建立局部笛卡尔坐标系
        {
            std::cout << "origin... lat=" << std::setprecision(12) << gnss_data.latitude << " lon=" << gnss_data.longitude << " alt=" << gnss_data.altitude << " heading=" << gnss_data.heading << std::endl;
            gnss_data.InitOriginPosition(); //给这个类一个所有的对象一个原点
            gnss_data.origin_position_inited = true;
        }
        purePursuit::Position point;
        gnss_data.GetOrientationMatrixFromYawAndLatLon(); //经纬度获取局部坐标
        point.x = gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3);
        point.y = gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3);
        point.heading = gnss_data.heading;
        wayPoints.push_back(point);
        //std::cout << " x= " << point.x << " y= " << point.y << " heading= " << point.heading << std::endl;
    }

    /*******************************************************************************/




    ros::Rate rate(10);

    //RRT::node *node1 = new RRT::node(0, 0);
    //mntRRT::node *node2 = new RRT::node(4, -5);
    std::shared_ptr<PathPublisher> RRT_path_pub_ptr = std::make_shared<PathPublisher>(nh, "RRT_path_pub", "/map", "/RRT_path", 100); //!!因为里面vector没办法清空，所以只能每次重新定义变量
    while (ros::ok())
    {
        ros::spinOnce();
        gnss_sub_ptr->ParseData(new_gnss_data_buff);
        grid_map_sub_ptr->ParseData(grid_map_data_buff); //将收到的点云数据push到cloud_data_buff的队尾
        ros::Time time_begin=ros::Time::now();
        //每次只处理队尾的点
        if (grid_map_data_buff.size() > 0&&new_gnss_data_buff.size() > 0) //把cloud里面的所有数据都处理干净
        {
            nav_msgs::OccupancyGrid grid_map_data = grid_map_data_buff.back();
            grid_map_data_buff.erase(grid_map_data_buff.begin(), grid_map_data_buff.end());

            // 把订阅的 经纬度拿出来
            GNSSData car_gnss_data = new_gnss_data_buff.front();
            new_gnss_data_buff.erase(new_gnss_data_buff.begin(), new_gnss_data_buff.end());
            car_gnss_data.GetOrientationMatrixFromYawAndLatLon(); //将经纬度转为局部北西天坐标系,这个是车辆坐标系相对于世界坐标系的转换矩阵

            vehiclePos.x = car_gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3);
            vehiclePos.y =  car_gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3);
            vehiclePos.heading =  car_gnss_data.heading;

            my_pure_pursuit.FindIndex(vehiclePos);
            nearestIndex = my_pure_pursuit.m_nearestIndex;
            foundIndex = my_pure_pursuit.m_foundIndex;
            //nearestIndex = foundIndex = 5;

            
            
            RRT::node start_p = RRT::node(0, 0);

            if (flag_path==0)  //  到达终点后变回0
            {
                vector<RRT::node> node_list;
                for (int i=nearestIndex; i<foundIndex;i++){
                    RRT::node tmp;
                    tmp.x = wayPoints[i].x;
                    tmp.y = wayPoints[i].y;
                    node_list.push_back(tmp);
                }

                
                global_to_vehicle(node_list,vehiclePos);
                flag_collision = RRT_ptr->check_collision_in_two_vector(node_list,grid_map_data);
                int tmpIndex = foundIndex + 20;
                end_p.x = wayPoints[tmpIndex].x;
                end_p.y = wayPoints[tmpIndex].y;


            }
     

            

            if (flag_collision)
            {
                flag_path = 1;
                flag_RRT_cmd = 1;

                if (!flag_RRT_end )
                {
                    if (!flag_replanning)
                    {
                        RRT::node end_point_vehicle =  global_to_vehicle_point( end_p, vehiclePos);
                        RRT_ptr->init(start_p, end_point_vehicle, grid_map_data);  // 车辆坐标系下start_p, end_p; grid_map_data是在激光雷达坐标系(车辆坐标系)下的地图。
                        RRT_path = RRT_ptr->GetPath(); // 车辆坐标系下的路径。 用全局变量来存储。
                        //转化为全局坐标系下的坐标
                        path_in_world = vehicle_to_global(RRT_path,vehiclePos);

                    }

                }
                cmd = Calculate(path_in_world,vehiclePos);
                if ( sqrt((vehiclePos.x-end_p.x)*(vehiclePos.x-end_p.x) + (vehiclePos.y-end_p.y)*(vehiclePos.y-end_p.y) < 0.5) )
                {
                    flag_RRT_end = false;
                    flag_replanning = false;
                    flag_path = 0;
                   // 发送到 pure_pursuit中进行融合处理。
                //  先把路径从车辆坐标系下转换到全局坐标系下，

                }
                else
                {
                    flag_RRT_end = false;
                    flag_replanning = true;
                }
              

            
            }else{
                cmd.steer = 0.0;
                cmd.speed = 0.0;
                flag_RRT_cmd = 0;
                flag_path = 0;
                flag_RRT_end = false;
                flag_replanning = false;
            }

            


            // 

            RRT_path_pub_ptr->PublishNavPath(path_in_world);
            grid_map_pub_ptr->Publish(RRT_ptr->vis_map_);
        }
        ros::Time time_end=ros::Time::now();
        double ros_duration=(time_end-time_begin).toSec();
        cout<<" RRT time ="<<ros_duration<<endl;
        rate.sleep();
    }
    return 0;
}
