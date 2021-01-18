/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#ifndef LIDAR_LOCALIZATION_GRID_MAP_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_GRID_MAP_PUBLISHER_HPP_
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <string>

namespace lidar_localization
{
    class GridMapPublisher
    {
    public:
        GridMapPublisher(ros::NodeHandle &nh,
                         std::string topic_name,
                         size_t buff_size,
                         std::string frame_id);
        void Publish(const nav_msgs::OccupancyGrid &map, double time);
        void Publish(const nav_msgs::OccupancyGrid &map);
        void PublishData(const nav_msgs::OccupancyGrid &map, ros::Time time);

        bool HasSubscribers();

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
        nav_msgs::OccupancyGrid map_;
    };
} // namespace lidar_localization
#endif