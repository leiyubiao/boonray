/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

//静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_longitude = 0.0; //作为当地坐标系的经纬度坐标
double lidar_localization::GNSSData::origin_latitude = 0.0;
double lidar_localization::GNSSData::origin_altitude = 0.0;
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter; //定义一个LocalCartesian的变量geo_converter

namespace lidar_localization
{
    void GNSSData::InitOriginPosition()
    {
        //std::cout<<"RESET"<<std::endl;
        geo_converter.Reset(latitude, longitude, altitude);

        origin_longitude = longitude;
        origin_latitude = latitude;
        origin_altitude = altitude;

        origin_position_inited = true;
    }

    void GNSSData::UpdateXYZ()
    {
        if (!origin_position_inited)
        {
            LOG(WARNING) << "GeoConverter has not set origin position";
        }
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
        //东北天转为北西天
    }

    //将gnss_data里面的rotationMatrixFromYawAndLatLonFloat(相对于车辆坐标系)的xyz转换到世界坐标系下的xyz
    void GNSSData::TransformPointInCarCoordinate_FxLy_ToWorld_NxWy(float &x, float &y)
    {
        if (!hasrotationMatrix)
        {
            GetOrientationMatrixFromYawAndLatLon();
        }
        float theta = heading / 180.0 * M_PI; //因为有一个求逆的动作，所以把这个负的角度变成正的了
        float x_tmp = rotationMatrixFromYawAndLatLonFloat(0, 3) + cosf(theta) * x + sinf(theta) * y;
        float y_tmp = rotationMatrixFromYawAndLatLonFloat(1, 3) - sinf(theta) * x + cosf(theta) * y;
        x = x_tmp;
        y = y_tmp;
    }

    //有问题

    /*测试代码
     GNSSData tmpGNSS;
            tmpGNSS.rotationMatrixFromYawAndLatLonFloat(0, 3) = 1.0f;
            tmpGNSS.rotationMatrixFromYawAndLatLonFloat(1, 3) = 1.0f;
            tmpGNSS.rotationMatrixFromYawAndLatLonFloat(2, 3) = 0.0f;
            car_gnss_data.TransformPointInCarCoordinate_FxLyUz_ToWorld_NxWySz(tmpGNSS);
    */
    void GNSSData::TransformPointInCarCoordinate_FxLyUz_ToWorld_NxWySz(GNSSData &gnss_data)
    {
        if (!hasrotationMatrix)
        {
            GetOrientationMatrixFromYawAndLatLon();
        }
        Eigen::Vector4f p1 = Eigen::Vector4f(gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3), gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3), gnss_data.rotationMatrixFromYawAndLatLonFloat(2, 3), 1.0f);
        Eigen::Matrix4f T_inverse = Eigen::Matrix4f::Identity(4, 4);
        T_inverse.block<3, 3>(0, 0) = rotationMatrixFromYawAndLatLonFloat.block<3, 3>(0, 0).transpose();
        T_inverse.block<3, 1>(0, 3) = -rotationMatrixFromYawAndLatLonFloat.block<3, 3>(0, 0).transpose() * rotationMatrixFromYawAndLatLonFloat.block<3, 1>(0, 3);
        Eigen::Vector4f p2 = T_inverse * p1;
        gnss_data.rotationMatrixFromYawAndLatLonFloat(0, 3) = p2(0);
        gnss_data.rotationMatrixFromYawAndLatLonFloat(1, 3) = p2(1);
        gnss_data.rotationMatrixFromYawAndLatLonFloat(2, 3) = p2(2);
    }

    void GNSSData::GetOrientationMatrixFromYawAndLatLon()
    {
        hasrotationMatrix = true;
        //std::cout<<"before local lat="<<latitude<<" lon="<<longitude<<" alt="<<altitude<<" heading="<<heading<<std::endl;
        //std::cout<<"before local E="<<local_E<<" N="<<local_N<<" U="<<local_U<<std::endl;
        geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
        //std::cout<<"after local E="<<local_E<<" N="<<local_N<<" U="<<local_U<<std::endl;

        //东北天坐标系
        /*
        rotationMatrixFromYawAndLatLonFloat(0, 3) = local_E; // 平移
        rotationMatrixFromYawAndLatLonFloat(1, 3) = local_N;
        rotationMatrixFromYawAndLatLonFloat(2, 3) = local_U;*/

        //北西天坐标系
        rotationMatrixFromYawAndLatLonFloat(0, 3) = local_N; // 平移
        rotationMatrixFromYawAndLatLonFloat(1, 3) = -local_E;
        rotationMatrixFromYawAndLatLonFloat(2, 3) = local_U;

        //前轴转后轴
        float theta = -heading * M_PI / 180.0;
        float L = 1.1;
        rotationMatrixFromYawAndLatLonFloat(0, 3) = rotationMatrixFromYawAndLatLonFloat(0, 3) - L * cos(theta);
        rotationMatrixFromYawAndLatLonFloat(1, 3) = rotationMatrixFromYawAndLatLonFloat(1, 3) - L * sin(theta);

        Eigen::AngleAxisf v1(M_PI / 4, Eigen::Vector3f(0, 0, -heading * M_PI / 180.0)); //沿z轴旋转heading，因为惯导是顺时针为正，所以这里需要负号
        rotationMatrixFromYawAndLatLonFloat.block<3, 3>(0, 0) = v1.toRotationMatrix();
        //std::cout<<rotationMatrixFromYawAndLatLonFloat<<std::endl;
    }

    bool GNSSData::SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time)
    {
        // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
        // 即找到与同步时间相邻的左右两个数据
        // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
        while (UnsyncedData.size() >= 2)
        {
            if (UnsyncedData.front().time > sync_time)
                return false;
            if (UnsyncedData.at(1).time < sync_time)
            {
                UnsyncedData.pop_front();
                continue;
            }
            if (sync_time - UnsyncedData.front().time > 0.2)
            {
                UnsyncedData.pop_front();
                break;
            }
            if (UnsyncedData.at(1).time - sync_time > 0.2)
            {
                UnsyncedData.pop_front();
                break;
            }
            break;
        }
        if (UnsyncedData.size() < 2)
            return false;

        GNSSData front_data = UnsyncedData.at(0);
        GNSSData back_data = UnsyncedData.at(1);
        GNSSData synced_data;

        double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
        double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
        synced_data.time = sync_time;
        synced_data.status = back_data.status;
        synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
        synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
        synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
        synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
        synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
        synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

        SyncedData.push_back(synced_data);

        return true;
    }
} // namespace lidar_localization