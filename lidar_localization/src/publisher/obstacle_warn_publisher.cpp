/*
 * @Description: 里程计信息发布
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:11:44
 */
#include "lidar_localization/publisher/obstacle_warn_publisher.hpp"

namespace lidar_localization
{
    ObstacleWarnPublisher::ObstacleWarnPublisher(ros::NodeHandle &nh,
                                                std::string topic_name,
                                                size_t buff_size,
                                                std::string frame_id)
        : nh_(nh), frame_id_(frame_id)
    {

        publisher_ = nh_.advertise<std_msgs::Int8>(topic_name, buff_size);
    }

    
    void ObstacleWarnPublisher::PublishObstacleWarning(const bool& hasObstacle)
    {
        if (hasObstacle)
        {
            //发送一个刹车信号
            std_msgs::Int8 msg;
            msg.data = 1;
            publisher_.publish(msg);
            ROS_WARN("obstacle detected");
        }
        else //没障碍物，发0
        {

            std_msgs::Int8 msg;
            msg.data = 0;
            publisher_.publish(msg);
        }
    }

} // namespace lidar_localization