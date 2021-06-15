#pragma once

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "vga_3d_camera/TempSrv.h"
#include "vga_3d_camera/Vga3DCameraConfig.h"

namespace terabee {

class Vga3DCamera {
 public:
  Vga3DCamera();
  virtual ~Vga3DCamera();

  void dynParamCallback(const terabee::Vga3DCameraConfig& config,
                        uint32_t level);
  void spin();

 private:
  void publishFrame(const terabee::cameraclient::Frame& f);
  void publishPointCloud(const terabee::cameraclient::Frame& f);
  pcl::PointCloud<pcl::PointXYZ> frame2pclPointcloud(
      const terabee::cameraclient::Frame& f);
  pcl::PointCloud<pcl::PointXYZ> getEmptyPointCloud(int cols, int rows);

  std::shared_ptr<terabee::cameraclient::ICameraClient> client_;

  ros::NodeHandle nh_;
  image_transport::Publisher frame_publisher_;
  ros::Publisher pointcloud_publisher_;

  dynamic_reconfigure::Server<terabee::Vga3DCameraConfig> server_;
  dynamic_reconfigure::Server<terabee::Vga3DCameraConfig>::CallbackType
      callback_function_;

  std::string ip_address_;
  int port_;
  std::string mode_;
  std::string id_frame_;
};

}  // namespace terabee
