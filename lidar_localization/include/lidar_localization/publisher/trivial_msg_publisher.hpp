/*
 * @Description: 在ros中发布IMU数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */
#ifndef TRIVIAL_MSG_PUBLISHER_HPP_
#define TRIVIAL_MSG_PUBLISHER_HPP_
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <string>
#include<std_msgs/Int8.h>

namespace lidar_localization
{
    template <class T>
    class TrivialMsgPublisher
    {
    public:
        TrivialMsgPublisher(ros::NodeHandle &nh,
                         std::string topic_name,
                         size_t buff_size,
                         std::string frame_id);
        void PublishObstacleWarning(const bool& hasObstacle);
      

    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frame_id_;
    };
} // namespace lidar_localization
#endif