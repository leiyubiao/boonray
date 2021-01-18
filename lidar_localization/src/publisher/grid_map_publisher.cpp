/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/grid_map_publisher.hpp"

namespace lidar_localization
{
    GridMapPublisher::GridMapPublisher(ros::NodeHandle & nh,
                                        std::string topic_name,
                                        size_t buff_size,
                                        std::string frame_id)
        : nh_(nh),frame_id_(frame_id)
    {

        publisher_ = nh_.advertise<nav_msgs::OccupancyGrid>(topic_name, buff_size);
        map_.header.frame_id = frame_id_;

    }

    void GridMapPublisher::Publish(const nav_msgs::OccupancyGrid &map, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(map, ros_time);
    }

    void GridMapPublisher::Publish(const nav_msgs::OccupancyGrid &map)
    {
        PublishData(map, ros::Time::now());
    }

    void GridMapPublisher::PublishData(const nav_msgs::OccupancyGrid &map, ros::Time time)
    {
        map_ = map;
        map_.header.stamp = time;
        publisher_.publish(map_);
    }

    bool GridMapPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
} // namespace lidar_localization