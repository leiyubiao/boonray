/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 12:58:10
 */
#ifndef LIDAR_LOCALIZATION_GRID_MAP_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_GRID_MAP_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include <nav_msgs/OccupancyGrid.h>


namespace lidar_localization {
class GridMapSubscriber {
  public:
    GridMapSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    GridMapSubscriber() = default;
    void ParseData(std::deque<nav_msgs::OccupancyGrid>& deque_gnss_data);

  private:
    void msg_callback(const nav_msgs::OccupancyGridConstPtr& gridmap_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<nav_msgs::OccupancyGrid> new_grid_map_data_;

    std::mutex buff_mutex_;
};
}
#endif