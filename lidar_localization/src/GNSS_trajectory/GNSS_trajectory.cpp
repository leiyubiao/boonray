#include "lidar_localization/GNSS_trajectory/GNSS_trajectory.hpp"
namespace lidar_localization
{
    TrajectoryOperator::TrajectoryOperator(string path) : csv_path_(path)
    {
    }
    /*nav_msgs::Path TrajectoryOperator::GeneratePath()
    {
        
        nav_msgs::Path traj;
        bool gnss_origin_position_inited=false;
        for (auto gnss_data : gnss_data_buff_)
        {
            

            if (!gnss_origin_position_inited) //以最开始受收到的经纬度为原点建立局部笛卡尔坐标系
            {
                std::cout << "origin... lat=" << gnss_data.latitude << " lon=" << gnss_data.longitude << " alt=" << gnss_data.altitude << " heading=" << gnss_data.heading << std::endl;
                gnss_data.InitOriginPosition(); //给这个类一个所有的对象一个原点
                gnss_origin_position_inited = true;
            }
            gnss_data.GetOrientationMatrixFromYawAndLatLon();
            Eigen::Matrix3f odometry_matrix = Eigen::Matrix3f::Identity(3, 3);
            odometry_matrix.block<3, 3>(0, 0) = gnss_data.rotationMatrixFromYawAndLatLonFloat.block<3, 3>(0, 0);

            Eigen::Quaterniond quaternion=odometry_matrix;

            geometry_msgs::PoseStamped this_pose_stamped;
            this_pose_stamped.pose.position.x = gnss_data.rotationMatrixFromYawAndLatLonFloat(0,3);
            this_pose_stamped.pose.position.y = gnss_data.rotationMatrixFromYawAndLatLonFloat(1,3);
            this_pose_stamped.pose.position.z = gnss_data.rotationMatrixFromYawAndLatLonFloat(2,3);
            thos_pose_stamped.pose.orientation().x=quaternion.x();
            thos_pose_stamped.pose.orientation().y=quaternion.y();
            thos_pose_stamped.pose.orientation().z=quaternion.z();
            thos_pose_stamped.pose.orientation().w=quaternion.w();
            this_pose_stamped.header.stamp = ros::time::now();
            this_pose_stamped.header.frame_id = "map";
            traj.header.stamp = ros::time::now();
            traj.header.frame_id = "map";
            traj.poses.push_back(this_pose_stamped);
        }

        return traj;
        
    }*/

    void TrajectoryOperator::ParseData(std::deque<GNSSData> &deque_GNSS_data_buff)
    {
        buff_mutex_.lock();

        if (gnss_data_buff_.size() > 0)
        {
            deque_GNSS_data_buff.insert(deque_GNSS_data_buff.end(), gnss_data_buff_.begin(), gnss_data_buff_.end());
            gnss_data_buff_.clear();
        }
        buff_mutex_.unlock();
    }
    void TrajectoryOperator::SaveCsv(std::deque<GNSSData> &waypoints)
    {
        // 写文件
        ofstream outFile;
        outFile.open(csv_path_, ios::out);
        outFile.setf(ios::fixed, ios::floatfield); // 设定为 fixed 模式，以小数点表示浮点数
        outFile.precision(12);                     // 设置精度 12
        //outFile<<"x"<<','<<"y"<<','<<"heading"<<endl;
        //float x =  position.x;
        while (waypoints.size() > 0)
        {
            auto p = waypoints.front();
            waypoints.pop_front();
            outFile << p.latitude << ',' << p.longitude << ',' << p.heading << endl;
        }

        outFile.close();
    }

    void TrajectoryOperator::ReadCsv()
    {
        //GNSSData pResult;
        ifstream fin(csv_path_); // 拷贝函数

        if (!fin)
        {
            ROS_ERROR("fail to open the file");
            exit(-1); //或者抛出异常。
        }
        else
        {
            ROS_INFO("open the file successfully");
        }

        string line;
        bool flag = true;
        GNSSData pre, cur;

        while (getline(fin, line))
        { // 整行读取，换行符"\n"区分，遇到文件尾标志eof终止读取。
            //cout<<"原始字符： "<<line<<endl; // 整行输出
            istringstream sin(line); // 将整行字符串line读入到字符串流istringstream中
            deque<string> Waypoints; // 声明一个字符串向量
            string info;
            
            while (getline(sin, info, ','))
            { // 将字符串流sin中的字符读入到waypoints字符串中，以逗号为分隔符。
                Waypoints.push_back(info);
            }
            // Get x,y,z
            string latitude_str = Waypoints[0];
            string longitude_str = Waypoints[1];
            string heading_str = Waypoints[2];

            double latitude, longitude, heading;
            stringstream slatitude, slongitude, sz;

            slatitude.precision(12);
            //slatitude.setf(ios::fixed, ios::doublefield);
            slongitude.precision(12);
            //slongitude.setf(ios::fixed, ios::doublefield);
            sz.precision(12);
            //sz.setf(ios::fixed, ios::doublefield);

            slatitude << latitude_str;
            slongitude << longitude_str;
            sz << heading_str;

            slatitude >> latitude;
            slongitude >> longitude;
            sz >> heading;

            cur.latitude = latitude;
            cur.longitude = longitude;
            cur.heading = heading;
            
            if (flag)
            {
                pre = cur;
                gnss_data_buff_.push_back(cur);
                flag = false;
            }

            if (Discrete(pre, cur))
            {
                pre = cur;
                //std::cout << "pre.x: " << pre.latitude << std::endl;
                gnss_data_buff_.push_back(cur);
            }
        }
        std::cout << " gnss read size=" << gnss_data_buff_.size() << std::endl;
    }

    bool TrajectoryOperator::Discrete(const GNSSData &pre, const GNSSData &cur)
    {
        GeographicLib::LocalCartesian geo_converter; //定义一个LocalCartesian的变量geo_converter
        geo_converter.Reset(pre.latitude, pre.longitude, pre.altitude);
        double dx = 0, dy = 0, dz = 0;
        geo_converter.Forward(cur.latitude, cur.longitude, cur.altitude, dx, dy, dz);
        double dist;
        dist = sqrt(dx * dx + dy * dy);
        if (dist > 0.2)
            return true; // 2.0 用到实际中时要改。
        return false;
    }

} // namespace lidar_localization
