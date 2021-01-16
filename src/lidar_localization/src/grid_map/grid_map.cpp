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
        resolution = 0.2;
        offsetx = 0.0;
        offsety = -5.0;
        limit_height = -0.2;
        limit_low = -0.2;
        kernal_di = 5;
        kernal_er = 3;
        height_diff = 0.3;
        voxelthreshold = 0.1;
        HeightUpper = 0.3;
        HeightLower = -0.3;

        cout << "width:" << width << ","
             << "height" << height << ","
             << "limit_low" << limit_low << endl;
        Map_width = width / resolution;
        Map_height = height / resolution;
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

    void GridMapConstructor::ConstructMapHeightDiff(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map)
    {

        vector<JudgeNode> nodelist(Map_width * Map_height);

        map.info.resolution = resolution;
        map.info.width = width / resolution;
        map.info.height = height / resolution;
        map.info.origin.position.x = offsetx;
        map.info.origin.position.y = offsety;
        map.data.resize((width / resolution) * (height / resolution));
        ROS_INFO("construct map");
        cout<<"cloud size= "<<PointCloudForMap->points.size()<<endl;
        for (size_t i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {
            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;
            if (x > offsetx && x < offsetx + width && y > offsety && y < offsety + height)
            {
                int x_ = (x - offsetx) / resolution; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码！！！！！！
                int y_ = (y - offsety) / resolution;
                JudgeNode nodetmp;
                if (nodetmp.visited == false)
                {
                    int index = x_ + y_ * (width / resolution);
                    nodetmp.x = x_;
                    nodetmp.y = y_;
                    nodetmp.max = z;
                    nodetmp.min = z;
                    nodetmp.index = index;
                    nodetmp.visited = true;
                    nodelist[index] = nodetmp;
                }
            }
        }

        for (size_t i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {
            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;
            if (x > offsetx && x < offsetx + width && y > offsety && y < offsety + height)
            {
                int x_ = (x - offsetx) / resolution; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码
                int y_ = (y - offsety) / resolution;

                int index = x_ + y_ * (width / resolution);
                if (z > nodelist[index].max)
                {
                    nodelist[index].max = z;
                }
                if (z < nodelist[index].min)
                {
                    nodelist[index].min = z;
                }
            }
        }

        for (int y = kernal_di; y < height / resolution - kernal_di; y++)
        {
            for (int x = kernal_di; x < width / resolution - kernal_di; x++)
            {
                int index = x + y * Map_width;
                if ((nodelist[index].max - nodelist[index].min) > height_diff)
                {
                    map.data[x + y * Map_width] = 100;
                }
            }
        }

        while (nodelist.size() > 0)
        {
            nodelist.pop_back();
        }
        
    }

    void GridMapConstructor::ConstructMapABSHeight(pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudForMap, nav_msgs::OccupancyGrid &map)
    {
        map.header.frame_id = "map";
        map.header.stamp = ros::Time::now();
        map.info.resolution = resolution;
        map.info.width = width / resolution;
        map.info.height = height / resolution;
        map.info.origin.position.x = offsetx;
        map.info.origin.position.y = offsety;
        map.data.resize((width / resolution) * (height / resolution));

        for (size_t i = 0; i < PointCloudForMap->size(); i++) //这里的i一定要赋值  不然得不到值
        {

            float x = PointCloudForMap->points[i].x;
            float y = PointCloudForMap->points[i].y;
            float z = PointCloudForMap->points[i].z;

            if (z > HeightLower && z < HeightUpper)
            {
                continue;
            }
            if (x > offsetx && x < offsetx + width && y > offsety && y < offsety + height)
            {
                int x_ = (x - offsetx) / resolution; //一定要先进行数据类型转换，转换为int否则，得到的索引值是乱码！！！！！！

                int y_ = (y - offsety) / resolution;
                int index = x_ + y_ * (width / resolution);
                map.data[index] = 254;
            }
        }
    }
} // namespace lidar_localization
