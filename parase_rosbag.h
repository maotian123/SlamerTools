#ifndef PARASE_ROSBAG_H
#define PARASE_ROSBAG_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <string>

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CompressedImage.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <std_msgs/Int8.h>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgcodecs.hpp"

struct VelodynePointXYZIRT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring,
                                                       ring)(float, time, time))

struct RsPointXYZIRT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  uint16_t ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint16_t, ring, ring)(double, timestamp, timestamp))


struct Pose4
{
    float x;
    float y;
    float z;
    float w;
};

struct Pose3
{
    float x;
    float y;
    float z;
};

struct ImuMsg
{
    Pose4 orientation;
    Pose3 angular_velocity;
    Pose3 linear_acceleration;
};

struct GpsMsg
{
    float latitude;
    float longitude;
    float altitude;
    int status;
    int sevice;
};

using PointXYZIRT = VelodynePointXYZIRT;



class parase_rosbag
{
private:
    std::vector<std::string> _bag_file;
    // ROS topic name
    // sensor_msgs/NavSatFix
    const std::string kDefaultFixTopic = "/gps/fix";
    // sensor_msgs/Imu
    // const std::string kDefaultImuTopic = "/lidar/imu";
    const std::string kDefaultImuTopic = "/gps/imu";
    // geometry_msgs/TwistStamped
    const std::string kDefaultTwistTopic = "/gps/twist";
    // sensor_msgs/PointCloud2
    const std::string kDefaultPointCloudTopic = "/rslidar_points";
    const std::string kDefaultPoseTopic = "/ndt_pose";
    // const std::string kDefaultPointCloudTopic = "/points_raw";
    // const std::string kDefaultCameraTopic =
    //     "/galaxy_camera_right/image_raw/compressed";
    // const std::string kDefaultCameraTopic =
    //     "/galaxy_camera_center/image_raw/compressed";
    const std::string kDefaultCameraTopic =
        "/galaxy_camera/image_raw/compressed";
        /* data */
    const std::string kPointCloudTimestampFileName = "point_clouds.ts";
    const std::string kImageTimestampFileName = "img.ts";

    const std::string kImuDataFileName   = "imu.ts";
    const std::string kGpsDataFileName   = "gps.ts";
    const std::string kTwistDataFileName = "twist.ts";
    const std::string kPoseDataFileName  = "pose.ts";

    int _lidar_index{0};
    int _camera_index{0};
    int _imu_index{0};
    int _gps_index{0};
    int _twist_index{0};
    int _ndt_pose_index{0};

    std::vector<uint64_t> _lidar_timestamps;
    std::vector<uint64_t> _camera_timestamps;
    std::vector<uint64_t> _imu_timestamps;
    std::vector<uint64_t> _pose_timestamps;
    std::vector<uint64_t> _gps_timestamps;
    std::vector<uint64_t> _seqs;

    uint64_t getTimeStamps(int32_t sec, int32_t nsec) {
        uint64_t time_stamp =
            (uint64_t)sec * 1000000 + (uint64_t)((uint64_t)nsec / 1000);
    return time_stamp;
    }
public:
    parase_rosbag(/* args */);
    void process();
    ~parase_rosbag();
private:
    void initFilePath();
    void processPointCloud(const std::string &bag_file);
    void processImage(const std::string &bag_file);
    void processImu(const std::string &bag_file);
    void processGps(const std::string &bag_file);
    void processTwist(const std::string &bag_file);
    void processPose(const std::string &bag_file);

    void writeLidarTimestamps();
    void writeCameraTimestamps();
};




#endif // PARASE_ROSBAG_H