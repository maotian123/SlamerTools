#include "parase_rosbag.h"


parase_rosbag::parase_rosbag(/* args */)
{
  initFilePath();
  process();
}

parase_rosbag::~parase_rosbag()
{
}

void parase_rosbag::initFilePath()
{
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-53-12_0.bag");
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-54-15_1.bag");
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-55-15_2.bag");
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-56-15_3.bag");
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-57-15_4.bag");
  _bag_file.push_back("/media/tyh/06889091889080BB/1/2023-03-08-14-58-15_5.bag");
}

void parase_rosbag::process()
{
  for (const auto& file : _bag_file) 
  {
    std::cout << "Parsing point cloud from: " << file << std::endl;
    processPointCloud(file);
  }
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

        std::string file_name = std::to_string(_index) + ".pcd";
        std::string file_path = "/media/tyh/06889091889080BB/bag_data/pcd";
        // pcl::io::savePCDFileASCII<PointXYZIRT>(file_path, *laser_cloud);
        pcl::io::savePCDFileBinary<PointXYZIRT>(file_path, *laser_cloud);
        _timestamps.push_back(timestamp);
        _seqs.push_back(seq);

        std::cout << "Save pcd file to " << file_path << std::endl;
        _index++;
      } else {
        std::cout << "Get empty point cloud record" << std::endl;
      }
    }
  }
}