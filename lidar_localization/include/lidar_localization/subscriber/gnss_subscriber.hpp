/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 12:58:10
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_GNSS_SUBSCRIBER_HPP_

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "gnss_pos/pos_xy.h"
#include "lidar_localization/sensor_data/gnss_data.hpp"

namespace lidar_localization
{
  class GNSSSubscriber
  {
  public:
    GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
    GNSSSubscriber() = default;
    void ParseData(std::deque<GNSSData> &deque_gnss_data);

  private:
    void msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);
    void msg_callback_gnss_pos(const gnss_pos::pos_xyConstPtr &pos_xy_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    ros::Subscriber subscriber_gnss_pos_;
    std::deque<GNSSData> new_gnss_data_;

    std::mutex buff_mutex_;
  };
} // namespace lidar_localization
#endif