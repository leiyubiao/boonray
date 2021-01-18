/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:56:27
 */
#include <iomanip>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include "glog/logging.h"

#include "lidar_localization/global_defination/global_defination.h"
#include "lidar_localization/tf_listener/tf_listener.hpp"
#include "lidar_localization/GNSS_trajectory/GNSS_trajectory.hpp"
#include "lidar_localization/publisher/path_publisher.hpp"
#include "lidar_localization/sensor_data/cloud_data.hpp"
#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "lidar_localization/path_tracking/purePursuit.hpp"
#include "lidar_localization/subscriber/gnss_subscriber.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h> //可视化
#include <geometry_msgs/PointStamped.h>
#include "lidar_localization/path_planning/RRT.hpp"

#include "lidar_localization/publisher/obstacle_warn_publisher.hpp"
#include "lidar_localization/publisher/grid_map_publisher.hpp"
#include "lidar_localization/subscriber/grid_map_subscriber.hpp"
#include "lidar_localization/obstacle_operation/AEB.hpp"

using namespace lidar_localization;

int aeb_flag = 0;


void AEB_callback(const std_msgs::Int8ConstPtr& msg)
{
    aeb_flag = msg->data;
    //cout<<"receive aeb command: "<<aeb_flag<<endl;
}

//geometry_msgs::PoseStamped rrt_cmd;
purePursuit::ctrlCommand rrt_cmd;
float flag_cmd;

void RRT_cmd_callback(const geometry_msgs::PoseStamped::ConstPtr& _rrt_cmd)
{
    rrt_cmd.speed = _rrt_cmd->pose.position.x;
    rrt_cmd.steer = _rrt_cmd->pose.position.y;
    rrt_cmd.brake = _rrt_cmd->pose.position.z;
    flag_cmd = _rrt_cmd->pose.orientation.x;
}

int main(int argc, char *argv[])
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "pathTracking_node");
    ros::NodeHandle nh;
    
    string csv_path = "/home/leiyubiao/boonraySJTU/src/lidar_localization/latlon_data/map.csv";
    std::shared_ptr<TrajectoryOperator> gnss_trajectory_ptr = std::make_shared<TrajectoryOperator>(csv_path);   //订阅惯导数据
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/huace_msg", 1000000); //订阅惯导数据
    ros::Publisher pub_command = nh.advertise<geometry_msgs::PoseStamped>("/car_command", 100);
    ros::Publisher pub_car_pos = nh.advertise<visualization_msgs::Marker>("/car_pos", 100);
    ros::Publisher pub_target_point = nh.advertise<geometry_msgs::PointStamped>("/car_target_point", 100);
    ros::Publisher pub_car_debug_point = nh.advertise<geometry_msgs::PointStamped>("/car_debug_point", 100);

    std::shared_ptr<PathPublisher> path_pub_ptr = std::make_shared<PathPublisher>(nh, "global_path_pub_in_tracking_node", "/map", "/global_path", 100);
    std::shared_ptr<PathPublisher> car_trajectory_pub_ptr = std::make_shared<PathPublisher>(nh, "car_trajectory_pub_in_tracking_node", "/map", "/global_path", 100);

    std::shared_ptr<RRT> RRT_ptr = std::make_shared<RRT>();
    std::shared_ptr<GridMapPublisher> grid_map_pub_ptr = std::make_shared<GridMapPublisher>(nh, "/RRT_debug_grid_map", 100, "map"); //定义一个发布删格地图的话题
    std::shared_ptr<GridMapSubscriber> grid_map_sub_ptr = std::make_shared<GridMapSubscriber>(nh, "/grid_map", 100);                //定义一个发布删格地图的话题
    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "global_map_points", "/map", 100);
    ros::Subscriber sub_AEB_=nh.subscribe<std_msgs::Int8>("/obstacle_detection", 1, AEB_callback);
    //ros::Publisher pub_command = nh.advertise<geometry_msgs::PoseStamped>("/rrt_car_command", 100);
    ros::Subscriber sub_rrt_cmd = nh.subscribe<geometry_msgs::PoseStamped>("/rrt_car_command",1,RRT_cmd_callback);
    //ros::Subscriber sub_speed2D_=nh.subscribe<std_msgs::Int8>("/huace_msg", 1, AEB_callback);

    std::deque<GNSSData> csv_gnss_data_buff;       //读取csv里的寻迹路径
    std::deque<GNSSData> new_gnss_data_buff;       //实时收到的惯导消息
    std::deque<GNSSData> car_trajectory_data_buff; //读取csv里的寻迹路径
    GNSSData car_gnss_data;

    gnss_trajectory_ptr->ReadCsv();                     //读取数据
    gnss_trajectory_ptr->ParseData(csv_gnss_data_buff); //将数据存入变量

    vector<purePursuit::Position> wayPoints;
    purePursuit::Position point;

    for (auto gnss_data : csv_gnss_data_buff)
    {
        if (!gnss_data.origin_position_inited) //以最开始受收到的经纬度为原点建立局部笛卡尔坐标系
        {
            std::cout << "origin... lat=" << std::setprecision(12) << gnss_data.latitude << " lon=" << gnss_data.longitude << " alt=" << gnss_data.altitude << " heading=" << gnss_data.heading << std::endl;
            gnss_data.InitOriginPosition(); //给这个类一个所有的对象一个原点
            gnss_data.origin_position_inited = true;
        }

        gnss_data.GetOrientationMatrixFromYawAndLatLon(); //经纬度获取局部坐标
        point.x = gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3);
        point.y = gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3);
        point.heading = gnss_data.heading;
        wayPoints.push_back(point);
        //std::cout << " x= " << point.x << " y= " << point.y << " heading= " << point.heading << std::endl;
    }

    purePursuit::PID_variables setPID;
    setPID.Kp_v = 1.1;
    setPID.Ki_v = 0.0;
    setPID.Kd_v = 0.5;
    setPID.Kp_brake = -5.0;
    setPID.Ki_brake = -0.0;
    setPID.Kd_brake = -2.0;
    float wheelBase = 1.1;
    float setIndex = 25; //往前瞄的点数
    //purePursuit pure_pusuit(wayPoints, wheelBase, setIndex);
    std::shared_ptr<purePursuit> pure_pursuit_ptr = std::make_shared<purePursuit>(wayPoints, wheelBase, setIndex,setPID);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        gnss_sub_ptr->ParseData(new_gnss_data_buff);
        //std::cout << "gnss size= " << new_gnss_data_buff.size() << std::endl;

        if (new_gnss_data_buff.size() > 0) //把cloud imu gnss里面的所有数据都处理干净
        {

            GNSSData car_gnss_data = new_gnss_data_buff.back();
            new_gnss_data_buff.erase(new_gnss_data_buff.begin(), new_gnss_data_buff.end());
            car_gnss_data.GetOrientationMatrixFromYawAndLatLon(); //将经纬度转为局部北西天坐标系,这个是车辆坐标系相对于世界坐标系的转换矩阵
            car_trajectory_data_buff.push_back(car_gnss_data);
            purePursuit::Position carPosInWorld(car_gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3), car_gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3), car_gnss_data.heading);
            purePursuit::ctrlCommand carCommand;
            float expected_v = 2.0f;
            float actual_v = car_gnss_data.speed2D;

            carCommand = pure_pursuit_ptr->VehicleControl(carPosInWorld,aeb_flag ,expected_v, actual_v);
            //carCommand = pure_pursuit_ptr->VehicleControl(carPosInWorld, aeb_flag);
            std::cout << "command calculated" << std::endl;

            //发布控制量
            geometry_msgs::PoseStamped command_ptr; //x存储速度，y存储转角
            command_ptr.pose.position.x = carCommand.speed;
            command_ptr.pose.position.y = carCommand.steer;
            if (abs(1-flag_cmd)<0.00001)//
            {
                command_ptr.pose.position.x = rrt_cmd.speed;
                command_ptr.pose.position.y = rrt_cmd.steer;
            }
            pub_command.publish(command_ptr);

            //发布轨迹
            path_pub_ptr->Publish(csv_gnss_data_buff);                 //整体轨迹
            car_trajectory_pub_ptr->Publish(car_trajectory_data_buff); //车辆实时轨迹
			cloud_pub_ptr->Publish_GPS2xy(csv_gnss_data_buff);
            //发布豫瞄点
            geometry_msgs::PointStamped target_point_pose; //x存储速度，y存储转角
            target_point_pose.point.x = pure_pursuit_ptr->m_globalPos.x;
            target_point_pose.point.y = pure_pursuit_ptr->m_globalPos.y;
            command_ptr.pose.position.z = carCommand.brake;
            target_point_pose.header.frame_id = "/map";
            pub_target_point.publish(target_point_pose);

            //发布车辆的坐标
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CUBE;
            // 设置marker的颜色
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;
            marker.pose.position.x = carPosInWorld.x;
            marker.pose.position.y = carPosInWorld.y;
            marker.pose.position.z = 0;

            //Eigen::Matrix3f rotate_M = car_gnss_data.rotationMatrixFromYawAndLatLonFloat.block<3, 3>(0, 0);
            //Eigen::Quaternionf quat = Eigen::Quaternionf(rotate_M);

            tf::Quaternion quat;
            quat.setRPY(0, 0, -car_gnss_data.heading / 180 * 3.1415926);
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            marker.pose.orientation.w = quat.w();

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            // 设置marker的大小
            marker.scale.x = 1.5;
            marker.scale.y = 0.8;
            marker.scale.z = 0.4;
            pub_car_pos.publish(marker);

            //验证车辆局部坐标系下的点转换到全局坐标系
            geometry_msgs::PointStamped car_debug_point; //x存储速度，y存储转角

            GNSSData pointInCarCoordinnate;
            pointInCarCoordinnate.rotationMatrixFromYawAndLatLonFloat(0, 3) = 1.0; //x
            pointInCarCoordinnate.rotationMatrixFromYawAndLatLonFloat(1, 3) = 1.0; //y
            pointInCarCoordinnate.rotationMatrixFromYawAndLatLonFloat(2, 3) = 0.0; //z

            /*将车里那个坐标系下的点转换到全局坐标系下************/
            float x = 1;
            float y = -2;
            car_gnss_data.TransformPointInCarCoordinate_FxLy_ToWorld_NxWy(x, y); //车辆坐标系下转到全局坐标系下
            /***********************************/

            car_debug_point.point.x = x;
            car_debug_point.point.y = y;

            car_debug_point.header.frame_id = "/map";
            pub_car_debug_point.publish(car_debug_point);
        }

        rate.sleep();
    }

    return 0;
}
