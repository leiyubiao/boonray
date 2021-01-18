/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 */
#include "lidar_localization/subscriber/grid_map_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization
{
    GridMapSubscriber::GridMapSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
        : nh_(nh)
    {
        subscriber_ = nh_.subscribe(topic_name, buff_size, &GridMapSubscriber::msg_callback, this);
    }

    void GridMapSubscriber::msg_callback(const nav_msgs::OccupancyGridConstPtr &gridmap_ptr)
    {
        std::lock_guard<std::mutex> my_lock_guard(buff_mutex_);
        nav_msgs::OccupancyGrid grid_map_data;

        grid_map_data.info.resolution = gridmap_ptr->info.resolution;
        grid_map_data.info.width = gridmap_ptr->info.width;
        grid_map_data.info.height = gridmap_ptr->info.height;
        grid_map_data.info.origin.position.x = gridmap_ptr->info.origin.position.x;
        grid_map_data.info.origin.position.y = gridmap_ptr->info.origin.position.y;
        grid_map_data.data = gridmap_ptr->data;



        new_grid_map_data_.push_back(grid_map_data);
    }

    void GridMapSubscriber::ParseData(std::deque<nav_msgs::OccupancyGrid> &grid_map_data_buff)
    {
        std::lock_guard<std::mutex> my_lock_guard(buff_mutex_);
        if (new_grid_map_data_.size() > 0)
        {
            grid_map_data_buff.insert(grid_map_data_buff.end(), new_grid_map_data_.begin(), new_grid_map_data_.end());
            new_grid_map_data_.clear();
        }
    }
} // namespace lidar_localization