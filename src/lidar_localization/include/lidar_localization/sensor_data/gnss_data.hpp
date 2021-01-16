/*
 * @Description: 
 * @Author: Ren Qian
 * @Date: 2019-07-17 18:25:13
 */
#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <deque>

#include "Geocentric/LocalCartesian.hpp"
#include <Eigen/Dense>
#include <iostream>
namespace lidar_localization
{
  class GNSSData
  {
  public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;
    double heading = 0.0;
    double speed2D = 0.0;
    int status = 0;
    int service = 0;
    Eigen::Matrix4f rotationMatrixFromYawAndLatLonFloat;

    static double origin_longitude; //静态变量，属于整个类里面的每一个对象
    static double origin_latitude;
    static double origin_altitude;

  private:
    static GeographicLib::LocalCartesian geo_converter;
    bool hasrotationMatrix;//标识是否有做过旋转矩阵

  public:
    GNSSData()
    {
      rotationMatrixFromYawAndLatLonFloat = Eigen::Matrix4f::Identity();
      hasrotationMatrix=false;
    }
    void InitOriginPosition();
    void UpdateXYZ();
    void GetOrientationMatrixFromYawAndLatLon();
    static bool SyncData(std::deque<GNSSData> &UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time);
    void TransformPointInCarCoordinate_FxLy_ToWorld_NxWy(float &x, float &y);
    void TransformPointInCarCoordinate_FxLyUz_ToWorld_NxWySz(GNSSData &gnss_data);
    static bool origin_position_inited;
  };
} // namespace lidar_localization
#endif