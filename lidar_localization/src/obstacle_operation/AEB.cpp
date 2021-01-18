/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */

#include "lidar_localization/obstacle_operation/AEB.hpp"

namespace lidar_localization
{
    //就用一个固定的车框来判断AEB
    void AEB_operator::LidarAEB(nav_msgs::OccupancyGrid &map, bool &hasObstacle) //根据输入的地图判断是否
    {
        hasObstacle = false;
        //cout<<"map origin x= "<<map.info.origin.position.x<<" y= "<<map.info.origin.position.y<<"  reso= "<<map.info.resolution<<" xN_="<<xN_<< " xF_="<<xF_<<" yL= "<<yL_<<" yR= "<<yR_<<endl;
        int grid_xN = (xN_ - map.info.origin.position.x) / map.info.resolution;
        int grid_xF = (xF_ - map.info.origin.position.x) / map.info.resolution;
        int grid_yL = (yL_ - map.info.origin.position.y) / map.info.resolution;
        int grid_yR = (yR_ - map.info.origin.position.y) / map.info.resolution;
        //cout << xN_ << " " << xF_ << " " << yL_ << " " << yR_ << endl;

        //cout << grid_xN << " " << grid_xF << " " << grid_yL << " " << grid_yR << endl;
        int max_x_index=max(grid_xN,grid_xF);
        int min_x_index=min(grid_xN,grid_xF);
        int max_y_index=max(grid_yL,grid_yR);
        int min_y_index=min(grid_yL,grid_yR);

        //cout << min_x_index << " " << max_x_index << " " << min_y_index << " " << max_y_index << endl;

        int max_index = map.info.width * map.info.height - 1;
        //cout << "max_index= " << max_index << endl;
        for (int x = min_x_index; x <= max_x_index ; ++x) 
            for (int y = min_y_index; y < max_y_index; ++y)
            {
                int index = x + y * map.info.width;

                if (index < max_index && index > 0)
                {
                    //cout << "index = " << index << endl;
                    //cout<<"map size="<<map.data.size()<<endl;
                    if (map.data[index] > 80)
                    {
                        hasObstacle = true;
                    }
                    map.data[index] = 70; //为了直观地展示出我们刹车的触发区域
                }
                else
                {
                    std::cout<<"out of grid map range"<<std::endl;
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

    //输入 删格地图，障碍物标识符，速度，前轮转角（左负右正），轴距
    void AEB_operator::LidarAEB_AckermanModel(nav_msgs::OccupancyGrid &map, bool &hasObstacle,float speed,float delta,float Wheelbase)
    {
        
        if(abs(delta)<3)
        {
            LidarAEB(map,hasObstacle);
            return ;
        }
        speed=abs(speed);
        float max_trajectory=4;
        float max_predict_t=max_trajectory/(speed+0.2);
        float L=Wheelbase;
        float R=L/tan(delta);//转弯半径
        float w=speed/R;//角速度
        int max_index = map.info.width * map.info.height - 1;
        for(float t=0;t<max_predict_t;t+=0.01)
        {
            float theta=w*t;
            float x=R*sin(theta);
            float y=-R*(1-cos(theta));
            cout<<"x= "<<x<<" y= "<<y;
            int x_index = (x - map.info.origin.position.x) / map.info.resolution;
            int y_index = (y - map.info.origin.position.y) / map.info.resolution;
            int index_map = x_index + y_index * map.info.width;
            if(index_map>0&&index_map<max_index)
                map.data[index_map]=70;
        }
        cout<<endl;
        
    }


} // namespace lidar_localization
