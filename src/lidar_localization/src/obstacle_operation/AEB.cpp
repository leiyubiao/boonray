/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */

#include "lidar_localization/obstacle_operation/AEB.hpp"

namespace lidar_localization
{

    void AEB_operator::LidarAEB(nav_msgs::OccupancyGrid &map, bool &hasObstacle) //根据输入的地图判断是否
    {
        hasObstacle = false;
        //ROS_INFO("hi");
        int grid_xN = (xN_ - map.info.origin.position.x) / map.info.resolution;
        int grid_xF = (xF_ - map.info.origin.position.x) / map.info.resolution;
        int grid_yL = (yL_ - map.info.origin.position.y) / map.info.resolution;
        int grid_yR = (yR_ - map.info.origin.position.y) / map.info.resolution;

        //cout << grid_xN << " " << grid_xF << " " << grid_yL << " " << grid_yR << endl;

        int max_index = map.info.width * map.info.height - 1;
        //cout << "max_index= " << max_index << endl;
        bool flag = true;
        for (int y = grid_xN; y <= grid_xF && flag; ++y)//因为车辆坐标系是前x左y，而栅格地图坐标系是右x前y
            for (int x = grid_yL; x < grid_yR; ++x)
            {
                int index = x + y * map.info.height;

                if (index < max_index && index > 0)
                {
                    //cout << "index = " << index << endl;
                    //cout<<"map size="<<map.data.size()<<endl;
                    if (map.data[index] != 100)
                    {
                        map.data[index] = 40; //为了直观地展示出我们刹车的触发区域
                    }
                    else
                    {
                        hasObstacle = true;
                        flag = false;
                        break;
                    }
                }
            }

        if (hasObstacle)
        {
            //发送一个刹车信号
            obstacle_msg_.data = 1;
        }
        else //没障碍物，发0
        {
            obstacle_msg_.data = 0;
        }
    }
} // namespace lidar_localization
