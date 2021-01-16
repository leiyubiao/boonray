/*
 * @Description: 通过ros发布点云
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "lidar_localization/publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace lidar_localization
{
    CloudPublisher::CloudPublisher(ros::NodeHandle &nh,
                                   std::string topic_name,
                                   std::string frame_id,
                                   size_t buff_size)
        : nh_(nh), frame_id_(frame_id)
    {
        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
    }

    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input, double time)
    {
        ros::Time ros_time((float)time);
        PublishData(cloud_ptr_input, ros_time);
    }

    void CloudPublisher::Publish(CloudData::CLOUD_PTR &cloud_ptr_input)
    {
        ros::Time time = ros::Time::now();
        PublishData(cloud_ptr_input, time);
    }

    void CloudPublisher::Publish_GPS2xy( std::deque<GNSSData> &gnss_data_buff, double time)
    {
        ros::Time ros_time((float)time);
        CloudData cloud_input; //保存全局地图点的坐标
        transformGPS2xy(gnss_data_buff, cloud_input);
        PublishData(cloud_input.cloud_ptr, ros_time);
    }

    void CloudPublisher::Publish_GPS2xy( std::deque<GNSSData> &gnss_data_buff)
    {
        ros::Time time = ros::Time::now();
        CloudData cloud_input; //保存全局地图点的坐标
        transformGPS2xy(gnss_data_buff, cloud_input);
        PublishData(cloud_input.cloud_ptr, time);
    }

    void CloudPublisher::transformGPS2xy( std::deque<GNSSData> &gnss_data_buff, CloudData &map_point_cloud_data)
    {

        bool gnss_origin_position_inited = false;
        for (auto gnss_data : gnss_data_buff)
        {
            if (!gnss_origin_position_inited) //以最开始受收到的经纬度为原点建立局部笛卡尔坐标系
            {
                std::cout << "origin... lat=" << gnss_data.latitude << " lon=" << gnss_data.longitude << " alt=" << gnss_data.altitude << " heading=" << gnss_data.heading << std::endl;
                gnss_data.InitOriginPosition(); //给这个类一个所有的对象一个原点
                gnss_origin_position_inited = true;
            }
            gnss_data.GetOrientationMatrixFromYawAndLatLon();//经纬度获取局部坐标
            pcl::PointXYZ tmp;
            tmp.x = gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3);
            tmp.y = gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3);
            tmp.z = gnss_data.rotationMatrixFromYawAndLatLonFloat(2, 3);
            map_point_cloud_data.cloud_ptr->points.push_back(tmp);
        }
    }
    void CloudPublisher::PublishData(CloudData::CLOUD_PTR &cloud_ptr_input, ros::Time time)
    {
        sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
        pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

        cloud_ptr_output->header.stamp = time;
        cloud_ptr_output->header.frame_id = frame_id_;
        publisher_.publish(*cloud_ptr_output);
    }

    bool CloudPublisher::HasSubscribers()
    {
        return publisher_.getNumSubscribers() != 0;
    }
} // namespace lidar_localization