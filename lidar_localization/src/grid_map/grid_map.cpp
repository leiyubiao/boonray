/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */

#include "lidar_localization/grid_map/grid_map.hpp"

namespace lidar_localization
{
    GridMapConstructor::GridMapConstructor()
    {

        width = 20.0;
        height = 10.0;
        grid_reso = 0.2;
        grid_origin_x = 0.0;
        grid_origin_y = -5.0;

        limit_height = -0.2;
        limit_low = -0.2;
        kernal_di = 5;
        kernal_er = 3;
        height_diff = 0.08;
        voxelthreshold = 0.1;
        HeightUpper = 0.3;
        HeightLower = -0.3;
        carHeight = 0.9;

        cout << "width:" << width << ","
             << "height" << height << ","
             << "limit_low" << limit_low << endl;
        grid_width = width / grid_reso;
        grid_height = height / grid_reso;
        max_index = grid_width * grid_height - 1; //栅格地图的最大索引号
        min_index = 0;
        car_width = 2; //车宽，用来膨胀栅格地图
        planning_radius=0.8;//再向外膨胀planning_radius米，方便路径规划的时候能够多远离障碍物一点
    }

    vector<GridMapConstructor::JudgeNode> GridMapConstructor::bresenham_k_lessthan_1(int x0, int y0, int x1, int y1)
    {

        int dx = x1 - x0;
        int dy = y1 - y0;

        vector<JudgeNode> node_vec;
        JudgeNode new_node = JudgeNode(x0, y0);
        node_vec.push_back(new_node);

        int p = 2 * dy - dx;
        int xi = x0;
        int yi = y0;

        for (xi = x0; xi <= x1; ++xi)
        {
            node_vec.push_back(JudgeNode(xi, yi));
            //计算下一个p
            if (p > 0)
            {
                p = p + 2 * (dy - dx);
                yi = yi + 1;
            }
            else
                p = p + 2 * dy;
        }
        return node_vec;
    }

    //所有象限测试，完全没有问题
    vector<GridMapConstructor::JudgeNode> GridMapConstructor::bresenham_check(int x0, int y0, int x1, int y1)
    {

        //将直线朝向为二三象限的点全部转换到一四象限
        if (x0 > x1)
        {
            int tmp = x0;
            x0 = x1;
            x1 = tmp;
            tmp = y1;
            y1 = y0;
            y0 = tmp;
        }

        int dy = y1 - y0;
        int dx = x1 - x0;

        vector<JudgeNode> node_vec;
        if (dy > 0) //表示一象限
        {
            if (abs(dx) > abs(dy)) //第一象限斜率小于1部分
            {
                node_vec = bresenham_k_lessthan_1(x0, y0, x1, y1);
                return node_vec;
            }

            else //第一象限斜率大于1部分
            {

                node_vec = bresenham_k_lessthan_1(y0, x0, y1, x1);
                for (unsigned int ii = 0; ii < node_vec.size(); ++ii)
                {
                    float tmp_x = node_vec[ii].x;
                    node_vec[ii].x = node_vec[ii].y;
                    node_vec[ii].y = tmp_x;
                }
                return node_vec;
            }
        }
        else //四象限
        {
            if (abs(dx) > abs(dy))
            {
                node_vec = bresenham_k_lessthan_1(x0, -1 * y0, x1, -1 * y1);
                for (unsigned int ii = 0; ii < node_vec.size(); ++ii)
                {
                    node_vec[ii].y = -1 * node_vec[ii].y;
                }
                return node_vec;
            }

            else
            {
                node_vec = bresenham_k_lessthan_1(-1 * y0, x0, -1 * y1, x1);
                for (unsigned int ii = 0; ii < node_vec.size(); ++ii)
                {
                    node_vec[ii].x = -1 * node_vec[ii].x;
                }

                for (unsigned int ii = 0; ii < node_vec.size(); ++ii)
                {
                    float tmp_x = node_vec[ii].x;
                    node_vec[ii].x = node_vec[ii].y;
                    node_vec[ii].y = tmp_x;
                }
                return node_vec;
            }
        }
    }

    //有碰撞，则返回true,经过测试，
    bool GridMapConstructor::check_collision_between_two_grid_point(float x1_InCarCoordinate, float y1_InCarCoordinate, float x2_InCarCoordinate, float y2_InCarCoordinate)
    {
        //从车辆坐标系得到栅格地图坐标
        JudgeNode p1_InGridMapCoordinate;
        JudgeNode p2_InGridMapCoordinate;
        get_grid_coordinate_from_car(x1_InCarCoordinate, y1_InCarCoordinate, p1_InGridMapCoordinate); //返回栅格地图的坐标
        get_grid_coordinate_from_car(x2_InCarCoordinate, y2_InCarCoordinate, p2_InGridMapCoordinate); //返回栅格地图的坐标

        vector<JudgeNode> node_vec = bresenham_check(p1_InGridMapCoordinate.x, p1_InGridMapCoordinate.y, p2_InGridMapCoordinate.x, p2_InGridMapCoordinate.y);
        bool HasCrossedGridMap; //判断得到的点是否超过栅格地图的范围，如果超过栅格地图的范围，则表示不会
        for (auto node_a : node_vec)
        {

            int index = get_index_from_grid(node_a, HasCrossedGridMap);
            if (HasCrossedGridMap) //表示点超过了栅格地图范围，则代表无障碍物
                continue;
            if (map_.data[index] >= WALKING_SPACE)
                return true;
        }
        return false;
    }

    void GridMapConstructor::get_grid_coordinate_from_car(float x, float y, GridMapConstructor::JudgeNode &grid_coordinate) //返回栅格地图的坐标
    {
        grid_coordinate.x = (x - grid_origin_x) / grid_reso;
        grid_coordinate.y = (y - grid_origin_y) / grid_reso;
    }

    //栅格地图索引得到栅格地图坐标
    bool GridMapConstructor::get_grid_coordinate_from_index(int grid_index, GridMapConstructor::JudgeNode *grid_coordinate) //由栅格地图index得到栅格地图坐标
    {
        bool HasCrossedGridMap = false;
        if (grid_index <= min_index || grid_index >= max_index)
            HasCrossedGridMap = true;

        grid_index = (grid_index <= min_index) ? min_index : grid_index;
        grid_index = (grid_index >= max_index) ? max_index : grid_index;

        grid_coordinate->y = grid_index % grid_width;
        grid_coordinate->x = grid_index - grid_coordinate->y * grid_width;
        return HasCrossedGridMap;
    }
    void GridMapConstructor::get_world_coordinate(const GridMapConstructor::JudgeNode *grid_coordinate, GridMapConstructor::JudgeNode *world_coordinate) //由栅格地图坐标得到车辆坐标系下坐标
    {
        world_coordinate->x = grid_coordinate->x * grid_reso + grid_origin_x;
        world_coordinate->y = grid_coordinate->y * grid_reso + grid_origin_y;
    }
    int GridMapConstructor::get_index_from_grid(const GridMapConstructor::JudgeNode &node, bool &HasCrossedGridMap)
    {
        HasCrossedGridMap = false;

        int index = node.y * grid_width + node.x;
        if (index <= min_index || index >= max_index)
            HasCrossedGridMap = true;
        //cout<<"index ="<<index<<endl;
        index = (index <= min_index) ? min_index : index;
        index = (index >= max_index) ? max_index : index;
        //cout<<"max index="<<m_maxRan<<"  height*width="<<grid_height*grid_width<<endl;
        return index;
    }

    void GridMapConstructor::filterVoxelGrid(const sensor_msgs::PointCloud2ConstPtr &Inpointcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr Outpointcloud, double th)
    {
        // Container for original & filtered data
        pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
        pcl::PCLPointCloud2 cloud_filtered; //该变量用于滤波的处理。
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_client(new pcl::PointCloud<pcl::PointXYZ>);

        // Convert to PCL data typet
        pcl_conversions::toPCL(*Inpointcloud, *cloud);

        // Perform the actual filtering
        pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
        // build the filter
        sor.setInputCloud(cloudPtr);
        sor.setLeafSize(th, th, th);
        // apply filter
        sor.filter(cloud_filtered);
        pcl::fromPCLPointCloud2(cloud_filtered, *Outpointcloud); ///将pcl::PointCloud格式转换成pcl::PointCloud<pcl::PointXYZ>为了后面将点取出进行姿态纠正
    }

    void GridMapConstructor::expand_map(nav_msgs::OccupancyGrid &map_to_be_expanded)
    {
        //根据车辆半径进行膨胀
        int radius = car_width / 2 / map_to_be_expanded.info.resolution;
        int max_index = map_to_be_expanded.info.width * map_to_be_expanded.info.height - 1;
        for (unsigned int xi = 0; xi < map_to_be_expanded.info.width; ++xi)
        {
            for (unsigned int yi = 0; yi < map_to_be_expanded.info.height; ++yi)
            {
                int index = xi + yi * map_to_be_expanded.info.width;
                //cout<<"index ="<<index<<endl;
                if ((index < max_index) && (index > 0) && map_to_be_expanded.data[index] >= LETHAL_OBSTACLE)
                {
                    //cout<<"expand"<<endl;
                    for (unsigned int m = xi - radius; m < xi + radius; ++m)
                    {
                        for (unsigned int n = yi - radius; n < yi + radius; ++n)
                        {
                            int index_new = m + n * map_to_be_expanded.info.width;
                            if ((index_new < max_index) && (index_new > 0))
                                map_to_be_expanded.data[index_new] = INSCRIBED_OBSTACLE;//根据车辆半径进行膨胀
                        }
                    }
                    map_to_be_expanded.data[index] = LETHAL_OBSTACLE;
                }
            }
        }
        
        //再向外膨胀1.0米，方便路径规划的时候能够多远离障碍物一点
        
        radius = planning_radius /map_to_be_expanded.info.resolution;
        for (unsigned int xi = 0; xi < map_to_be_expanded.info.width; ++xi)
        {
            for (unsigned int yi = 0; yi < map_to_be_expanded.info.height; ++yi)
            {
                int index = xi + yi * map_to_be_expanded.info.width;
                //cout<<"index ="<<index<<endl;
                if ((index < max_index) && (index > 0) && map_to_be_expanded.data[index] >= INSCRIBED_OBSTACLE)
                {
                    //cout<<"expand"<<endl;
                    for (unsigned int m = xi - radius; m < xi + radius; ++m)
                    {
                        for (unsigned int n = yi - radius; n < yi + radius; ++n)
                        {
                            int index_new = m + n * map_to_be_expanded.info.width;
                            if ((index_new < max_index) && (index_new > 0))
                                map_to_be_expanded.data[index_new] = WALKING_SPACE;//根据车辆半径进行膨胀
                        }
                    }
                    map_to_be_expanded.data[index] = INSCRIBED_OBSTACLE;
                }
            }
        }
    }

    void GridMapConstructor::ConstructMapHeightDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map)
    {
        JudgeNode  node_init;
        vector<JudgeNode> nodelist(grid_width * grid_height,node_init);
        
        map.info.resolution = grid_reso;
        map.info.width = width / grid_reso;
        map.info.height = height / grid_reso;
        map.info.origin.position.x = grid_origin_x;
        map.info.origin.position.y = grid_origin_y;
        map.data.resize((width / grid_reso) * (height / grid_reso));
        ROS_INFO("construct map");
        cout << "cloud size= " << PointCloudForMap->points.size() << endl;

        //先找出每格的最低点的z值
        for (unsigned int i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {
            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;
            if (z < carHeight && x > grid_origin_x && x < grid_origin_x + width && y > grid_origin_y && y < grid_origin_y + height) //carHeight去除很高点云
            {
                if(x<2.4&&y<1.4&&y>-1.4)    continue;//将车框过滤掉
                int x_ = (x - grid_origin_x) / grid_reso; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码！！！！！！
                int y_ = (y - grid_origin_y) / grid_reso;
                int index = x_ + y_ * (width / grid_reso);
                if(nodelist[index].min_z>z)
                {
                   nodelist[index].min_z=z;
                }
                
            }
        }

        for (unsigned int i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {
            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;
            if (z < carHeight &&x > grid_origin_x && x < grid_origin_x + width && y > grid_origin_y && y < grid_origin_y + height)
            {
                if(x<2.4&&y<1.4&&y>-1.4)    continue;
                int x_ = (x - grid_origin_x) / grid_reso; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码
                int y_ = (y - grid_origin_y) / grid_reso;
                int index = x_ + y_ * (width / grid_reso);

                if (z > nodelist[index].min_z+height_diff)
                {
                    nodelist[index].higherHeightDiffThanMinZNumber++;
                }
                
            }
        }

        //根据高程差建地图
        int max_count=6;//当满足higherHeightDiffThanMinZNumber数量大于max_count后，表示有障碍物

        for (int y = kernal_di; y < height / grid_reso - kernal_di; y++)
        {
            for (int x = kernal_di; x < width / grid_reso - kernal_di; x++)
            {
                int index = x + y * grid_width;
                if (nodelist[index].higherHeightDiffThanMinZNumber>max_count )
                {
                    map.data[x + y * grid_width] = LETHAL_OBSTACLE;
                }
            }
        }

        expand_map(map);

        float xN_=0;
        float xF_=1.8;
        float yL_=1;
        float yR_=-1;

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
        for (int x = min_x_index; x <= max_x_index ; ++x) 
            for (int y = min_y_index; y < max_y_index; ++y)
            {
                int index = x + y * map.info.width;

                if (index < max_index && index > 0)
                {
                   
                map.data[index] = FREE_SPACE;
                }
                
            }


        while (nodelist.size() > 0)
        {
            nodelist.pop_back();
        }
        map_ = map;
    }

    void GridMapConstructor::ConstructMapABSHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map)
    {
        map.header.frame_id = "map";
        map.header.stamp = ros::Time::now();
        map.info.resolution = grid_reso;
        map.info.width = width / grid_reso;
        map.info.height = height / grid_reso;
        map.info.origin.position.x = grid_origin_x;
        map.info.origin.position.y = grid_origin_y;
        map.data.resize((width / grid_reso) * (height / grid_reso));

        for (unsigned int i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {

            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;

            if (z > HeightLower && z < HeightUpper)
            {
                continue;
            }
            if (x > grid_origin_x && x < grid_origin_x + width && y > grid_origin_y && y < grid_origin_y + height)
            {
                int x_ = (x - grid_origin_x) / grid_reso; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码！！！！！！

                int y_ = (y - grid_origin_y) / grid_reso;
                int index = x_ + y_ * (width / grid_reso);
                map.data[index] = LETHAL_OBSTACLE;
            }
        }
    }
} // namespace lidar_localization
