/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-03-31 13:10:51
 */
#include "lidar_localization/subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace lidar_localization {
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {
    //subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
    subscriber_gnss_pos_=nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback_gnss_pos, this);//订阅我们自己的gnss话题
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    buff_mutex_.lock();
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
    buff_mutex_.unlock();
}

void GNSSSubscriber::msg_callback_gnss_pos(const gnss_pos::pos_xyConstPtr& pos_xy_ptr) {

    buff_mutex_.lock();
    GNSSData gnss_data;
    gnss_data.time = pos_xy_ptr->header.stamp.toSec();
    gnss_data.latitude = pos_xy_ptr->latitude;
    gnss_data.longitude = pos_xy_ptr->longitude;
    gnss_data.altitude = pos_xy_ptr->altitude;
    gnss_data.heading = pos_xy_ptr->heading;
    gnss_data.speed2D = pos_xy_ptr->speed2D;
    //std::cout << " lat=" << gnss_data.latitude << " lon=" << gnss_data.longitude << " alt=" << gnss_data.altitude << " heading=" << gnss_data.heading << std::endl;

    new_gnss_data_.push_back(gnss_data);
    buff_mutex_.unlock();
}

void GNSSSubscriber::ParseData(std::deque<GNSSData>& gnss_data_buff) {
    buff_mutex_.lock();
    if (new_gnss_data_.size() > 0) {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
    buff_mutex_.unlock();
}
}