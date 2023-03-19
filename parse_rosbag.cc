#include "parase_rosbag.h"


parase_rosbag::parase_rosbag(/* args */)
{
  initFilePath();
}

parase_rosbag::~parase_rosbag()
{
}

void parase_rosbag::initFilePath()
{
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-53-12_0.bag");
  // _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-54-15_1.bag");
  // _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-55-15_2.bag");
  // _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-56-15_3.bag");
  // _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-57-15_4.bag");
  // _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-58-15_5.bag");
}

void parase_rosbag::process()
{
  std::cout << " _bag_file size " << _bag_file.size() << std::endl;
  for (const auto& file : _bag_file) 
  {
    std::cout << "Parsing point cloud from: " << file << std::endl;
    // processPointCloud(file);
    // processImage(file);
    // processImu(file);
    processGps(file);
    processTwist(file);
    processPose(file);
  }

  writeLidarTimestamps();
  writeCameraTimestamps();
}

void parase_rosbag::processPointCloud(const std::string& bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }

  std::vector<std::string> topics;
  topics.push_back(kDefaultPointCloudTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
    if (topic == kDefaultPointCloudTopic) {
      sensor_msgs::PointCloud2::ConstPtr cloud_msg =
          m.instantiate<sensor_msgs::PointCloud2>();
      if (cloud_msg != NULL) {
        sensor_msgs::PointCloud2 current_cloud_msg = std::move(*cloud_msg);
        ros::Time time = current_cloud_msg.header.stamp;
        uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
        uint64_t seq = current_cloud_msg.header.seq;
        pcl::PointCloud<RsPointXYZIRT>::Ptr rs_cloud;
        pcl::PointCloud<PointXYZIRT>::Ptr laser_cloud;
        rs_cloud.reset(new pcl::PointCloud<RsPointXYZIRT>());
        laser_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
        pcl::moveFromROSMsg(current_cloud_msg, *rs_cloud);
        // remove NaN points
        if (!rs_cloud->is_dense) {
          std::vector<int> indices;
          pcl::removeNaNFromPointCloud(*rs_cloud, *rs_cloud, indices);
          rs_cloud->is_dense = true;
        }
        // Convert Robosense cloud to velodyne cloud.
        laser_cloud->points.resize(rs_cloud->size());
        laser_cloud->is_dense = rs_cloud->is_dense;
        laser_cloud->width = rs_cloud->width;
        laser_cloud->height = rs_cloud->height;
        double t0 = 0;
        for (size_t i = 0; i < rs_cloud->size(); i++) {
          auto& src = rs_cloud->points[i];
          auto& dst = laser_cloud->points[i];
          if (i == 0) {
            t0 = src.timestamp;
          }
          dst.x = src.x;
          dst.y = src.y;
          dst.z = src.z;
          dst.intensity = (float)src.intensity;
          dst.ring = src.ring;
          dst.time = (float)(src.timestamp - t0);
        }

        std::string file_name = std::to_string(_lidar_index) + ".pcd";
        std::string file_path = "/media/tyh/06889091889080BB/bag_data/pcd/";
        // pcl::io::savePCDFileASCII<PointXYZIRT>(file_path, *laser_cloud);
        pcl::io::savePCDFileBinary<PointXYZIRT>(file_path  + file_name, *laser_cloud);
        _lidar_timestamps.push_back(timestamp);
        _seqs.push_back(seq);

        std::cout << "Save pcd file to " << file_path + file_name << std::endl;
        _lidar_index++;
      } else {
        std::cout << "Get empty point cloud record" << std::endl;
      }
    }
  }

  bag.close();
}

void parase_rosbag::processImage(const std::string &bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(kDefaultCameraTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
    if (topic == kDefaultCameraTopic) {
      sensor_msgs::CompressedImage::ConstPtr image_msg =
          m.instantiate<sensor_msgs::CompressedImage>();
      if (image_msg != NULL) {
        uint64_t seq = image_msg->header.seq;
        cv::Mat image;
        try {
          cv::imdecode(cv::Mat(image_msg->data), cv::IMREAD_COLOR, &image);
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("Could not decode image with type %s",
                    image_msg->format.c_str());
          std::cout << "Could not decode image for sequence: " << seq << std::endl;
        }

        ros::Time time = image_msg->header.stamp;
        uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
        
        std::string file_name = std::to_string(_camera_index) + ".jpg";
        std::string file_path = "/media/tyh/06889091889080BB/bag_data/img/";
        std::string path = file_path + file_name;
        if (!cv::imwrite(path, image)) {
          
          std::cout << "Failed to write image: " << file_path + file_name << std::endl;
          continue;
        }

        _camera_timestamps.push_back(timestamp);
      
        std::cout << "Save image file to " << file_path + file_name << std::endl;
        
      } else {
        std::cout << "Get empty iamge record";
      }
    }

    _camera_index++;
  }
  bag.close();
}

void parase_rosbag::processImu(const std::string &bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(kDefaultImuTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/imu/" + kImuDataFileName;
  std::ofstream file(ts_file_path, std::ios::trunc); 
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
       if (topic == kDefaultImuTopic) {
      sensor_msgs::Imu::ConstPtr imu_msgs =
          m.instantiate<sensor_msgs::Imu>();
      if (imu_msgs != NULL) {
        ros::Time time = imu_msgs->header.stamp;
        uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
        
        if(file.is_open())
        {
          file << _imu_index << "，" 
               << timestamp  << "，"
               << imu_msgs->angular_velocity.x  << "，" 
               << imu_msgs->angular_velocity.y  << "，" 
               << imu_msgs->angular_velocity.z  << "，" 
               << imu_msgs->linear_acceleration.x  << "，" 
               << imu_msgs->linear_acceleration.y  << "，" 
               << imu_msgs->linear_acceleration.z  << "，" 
               << imu_msgs->orientation.x  << "，" 
               << imu_msgs->orientation.y  << "，" 
               << imu_msgs->orientation.z  << "，" 
               << imu_msgs->orientation.w  << "，" << std::endl;
        }
      }
    }
    _imu_index++;
  }
  bag.close();
  file.close();
}

void parase_rosbag::processGps(const std::string &bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(kDefaultFixTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/gps/" + kGpsDataFileName;
  std::ofstream file(ts_file_path, std::ios::trunc); 
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
       if (topic == kDefaultFixTopic) {
      sensor_msgs::NavSatFix::ConstPtr gps_msgs = 
          m.instantiate<sensor_msgs::NavSatFix>();
    
      if (gps_msgs != NULL) {
        ros::Time time = gps_msgs->header.stamp;
        uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
        
        if(file.is_open())
        {
          file << _gps_index << "，" << timestamp  << "，"
               << gps_msgs->altitude << "，" 
               << gps_msgs->latitude  << "，" 
               << gps_msgs->longitude  << "，" 
               << (int)gps_msgs->status.status  << "，"  << std::endl;
        }
      }
    }
    _gps_index++;
  }
  bag.close();
  file.close();
}

void parase_rosbag::processTwist(const std::string &bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(kDefaultTwistTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/twist/" + kTwistDataFileName;
  std::ofstream file(ts_file_path, std::ios::trunc); 
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
      if (topic == kDefaultTwistTopic) {
        geometry_msgs::TwistStamped::ConstPtr twist_msgs = 
            m.instantiate<geometry_msgs::TwistStamped>();
        if (twist_msgs != NULL) {
          ros::Time time = twist_msgs->header.stamp;
          uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
          
        if(file.is_open())
        {
          file << _twist_index << "，" << timestamp  << "，"
               << twist_msgs->twist.angular.x << "，" 
               << twist_msgs->twist.angular.y << "，" 
               << twist_msgs->twist.angular.z << "，" 
               << twist_msgs->twist.linear.x  << "，"  
               << twist_msgs->twist.linear.y  << "，"  
               << twist_msgs->twist.linear.z  << "，"  
               << std::endl;
        }
      }
    }
    _gps_index++;
  }
  bag.close();
  file.close();
}

void parase_rosbag::processPose(const std::string &bag_file)
{
  rosbag::Bag bag;
  try 
  {
    bag.open(bag_file, rosbag::bagmode::Read);
  }
  catch (rosbag::BagException e) 
  {
    std::cout << "Open ros bag file error: " << e.what() << std::endl;
    return;
  }
  std::vector<std::string> topics;
  topics.push_back(kDefaultPoseTopic);
  rosbag::View view(bag, rosbag::TopicQuery(topics));
  
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/pose/" + kPoseDataFileName;
  std::ofstream file(ts_file_path, std::ios::trunc); 
  for (const rosbag::MessageInstance& m : view) {
    std::string topic = m.getTopic();
      if (topic == kDefaultPoseTopic) {
        geometry_msgs::PoseStamped::ConstPtr pose_msgs = 
            m.instantiate<geometry_msgs::PoseStamped>();
        if (pose_msgs != NULL) {
          ros::Time time = pose_msgs->header.stamp;
          uint64_t timestamp = getTimeStamps(time.sec, time.nsec);
          
        if(file.is_open())
        {
          file << _ndt_pose_index << "，" 
               << timestamp  << "，"
               << pose_msgs->pose.position.x << "，" 
               << pose_msgs->pose.position.y << "，" 
               << pose_msgs->pose.position.z << "，" 
               << pose_msgs->pose.orientation.x  << "，"  
               << pose_msgs->pose.orientation.y  << "，"  
               << pose_msgs->pose.orientation.z  << "，"  
               << pose_msgs->pose.orientation.w  << "，"  
               << std::endl;
        }
      }
    }
    _ndt_pose_index++;
  }
  bag.close();
  file.close();
}

void parase_rosbag::writeLidarTimestamps()
{
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/pcd/timestamp/" + kPointCloudTimestampFileName;
  std::ofstream file(ts_file_path, std::ios::trunc);
  if(file.is_open())
  {
    for(size_t i = 0; i < _lidar_timestamps.size(); ++i)
    {
      file << _lidar_timestamps[i] << std::endl;
    }
    file.close();
  }
}

void parase_rosbag::writeCameraTimestamps()
{
  std::string ts_file_path = "/media/tyh/06889091889080BB/bag_data/img/timestamp/" + kImageTimestampFileName;
  std::ofstream file(ts_file_path, std::ios::trunc);
  if(file.is_open())
  {
    for(size_t i = 0; i < _camera_timestamps.size(); ++i)
    {
      file << _camera_timestamps[i] << std::endl;
    }
    file.close(); 
  }
}


