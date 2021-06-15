#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <mutex>
#include <string>

// Temperature message
#include <sensor_msgs/Temperature.h>

#include <CameraClient/ClientFactory.hpp>
#include <CameraClient/ICameraClient.hpp>

#include "Vga3DCamera/Vga3DCamera.hpp"

// cam parameters
constexpr int IR_PX_SCALE = 32;

namespace terabee {

Vga3DCamera::Vga3DCamera() {
  // Get parameters
  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.param("ip_address_", ip_address_,
                             std::string("192.168.1.153"));
  private_node_handle_.param("port_", port_, std::stoi("8080"));
  private_node_handle_.param("mode_", mode_, std::string("custom19"));
  private_node_handle_.param("id_frame_", id_frame_, std::string("map"));

  // Publishers
  image_transport::ImageTransport it(nh_);
  frame_publisher_ = it.advertise("/vga_3d_camera/image", 1);
  pointcloud_publisher_ =
      nh_.advertise<pcl::PCLPointCloud2>("/vga_3d_camera/pointcloud", 1);
  client_ = terabee::cameraclient::ClientFactory::createCameraClient(
      ip_address_, port_);

  // Output loaded parameters to console for double checking
  ROS_INFO("[%s] is up and running with the following parameters:",
           ros::this_node::getName().c_str());
  ROS_INFO("[%s] ip address: %s", ros::this_node::getName().c_str(),
           ip_address_.c_str());
  ROS_INFO("[%s] port: %s", ros::this_node::getName().c_str(),
           std::to_string(port_).c_str());
  ROS_INFO("[%s] mode: %s", ros::this_node::getName().c_str(), mode_.c_str());
  ROS_INFO("[%s] id_frame: %s", ros::this_node::getName().c_str(),
           id_frame_.c_str());

  // Dynamic reconfigure
  callback_function_ =
      boost::bind(&Vga3DCamera::dynParamCallback, this, _1, _2);
}

Vga3DCamera::~Vga3DCamera() {}

void Vga3DCamera::dynParamCallback(const terabee::Vga3DCameraConfig& config,
                                   uint32_t level) {
  client_->requestStopStream();
  if (client_->setCameraMode(mode_)) {
    ROS_INFO("[%s] is setting mode to: %s", ros::this_node::getName().c_str(),
             mode_.c_str());
  }
  if (config.set_stream_fps > 0) {
    ROS_INFO("[%s] is setting fps to: %s", ros::this_node::getName().c_str(),
             std::to_string(config.set_stream_fps).c_str());
    client_->setStreamFps(config.set_stream_fps);
  }
  if (config.set_stream_frame_type > 0) {
    ROS_INFO("[%s] is setting Frame Type to: %s",
             ros::this_node::getName().c_str(),
             std::to_string(config.set_stream_frame_type).c_str());
    client_->setStreamFrameType(
        static_cast<terabee::cameraclient::Frame::FrameType>(
            config.set_stream_frame_type));
  }
  if (config.set_undistortion_type > 0) {
    ROS_INFO("[%s] is setting Undistortion Type to: %s",
             ros::this_node::getName().c_str(),
             std::to_string(config.set_undistortion_type).c_str());
    client_->setUndistortionType(
        static_cast<terabee::cameraclient::ICameraClient::UndistortionType>(
            config.set_undistortion_type));
  }

  if (config.set_gamma_correction > 0) {
    ROS_INFO("[%s] is setting IR gamma correction to: %s",
             ros::this_node::getName().c_str(),
             std::to_string(config.set_gamma_correction).c_str());
    client_->setIrGammaCorrection(config.set_gamma_correction);
  }

  if (config.set_small_signal_threshold > 0) {
    ROS_INFO("[%s] is setting small signal threshold to: %s",
             ros::this_node::getName().c_str(),
             std::to_string(config.set_small_signal_threshold).c_str());
    client_->setSmallSignalThreshold(
        config.set_small_signal_threshold,
        terabee::cameraclient::ICameraClient::SSRThresholdRange::RANGE_2048);
  }

  if (!client_->validateConnection()) {
    // To avoid to requestStartStream when the dynParamCallback is set in the
    // constructor
    return;
  }
  uint16_t stream_port = port_ + 1;
  if (!client_->requestStartStream(stream_port)) {
    ROS_WARN("Failed to request start stream on port: %s",
             std::to_string(stream_port).c_str());
    return;
  }
}

void Vga3DCamera::spin() {
  client_->connect();
  if (!client_->validateConnection()) {
    ROS_INFO("Failed to validate connection");
    return;
  }
  std::mutex m;
  client_->registerOnFrameUpdateCallback(
      [this, &m](const terabee::cameraclient::Frame& f) {
        std::lock_guard<std::mutex> l(m);
        publishFrame(f);
        publishPointCloud(f);
      });
  server_.setCallback(callback_function_);

  while (ros::ok()) {
    ros::spinOnce();
  }
  client_->requestStopStream();
  client_->disconnect();
}

void Vga3DCamera::publishFrame(const terabee::cameraclient::Frame& f) {
  cv::Mat cv_frame;
  if (f.type == terabee::cameraclient::Frame::FrameType::DEPTH) {
    cv_frame.create(VGA_Y_SIZE, VGA_X_SIZE, CV_16U);
    std::transform(f.data.begin(), f.data.end(), (uint16_t*)cv_frame.data,
                   [](uint16_t p) { return p * 16; });
  }
  if (f.type == terabee::cameraclient::Frame::FrameType::IR) {
    cv_frame.create(VGA_Y_SIZE, VGA_X_SIZE, CV_16U);
    std::transform(f.data.begin(), f.data.end(), (uint16_t*)cv_frame.data,
                   [](uint16_t p) -> uint8_t {
                     return static_cast<uint8_t>((p * IR_PX_SCALE) / 256);
                   });
  }
  if (f.type == terabee::cameraclient::Frame::FrameType::DEPTH_IR) {
    cv_frame.create(VGA_Y_SIZE, 2 * VGA_X_SIZE, CV_16U);
    for (size_t h = 0; h < VGA_Y_SIZE; h++) {
      for (size_t w = 0; w < VGA_X_SIZE; w++) {
        // Depth
        cv_frame.at<uint16_t>(h, w) = f.data[h * VGA_X_SIZE + w] * 16;
        // IR
        auto data = f.data[h * VGA_X_SIZE + w + f.singleFrameSize()];
        cv_frame.at<uint16_t>(h, w + VGA_X_SIZE) = data * IR_PX_SCALE;
      }
    }
  }

  sensor_msgs::ImagePtr image_msg =
      cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_frame).toImageMsg();
  frame_publisher_.publish(image_msg);
}

void Vga3DCamera::publishPointCloud(const terabee::cameraclient::Frame& f) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  if (f.type == terabee::cameraclient::Frame::FrameType::DEPTH)
    pointCloud = frame2pclPointcloud(f);
  else
    return;
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(pointCloud, pcl_msg);
  pcl_msg.header.frame_id = id_frame_;
  pointcloud_publisher_.publish(pcl_msg);
}

pcl::PointCloud<pcl::PointXYZ> Vga3DCamera::frame2pclPointcloud(
    const terabee::cameraclient::Frame& f) {
  pcl::PointCloud<pcl::PointXYZ> cloud =
      getEmptyPointCloud(VGA_X_SIZE, VGA_Y_SIZE);
  terabee::cameraclient::PointCloud pc = client_->frame2PointCloud(f);
  for (size_t i = 0; i < pc.size(); i++) {
    const terabee::cameraclient::Point& p = pc.points.at(i);
    cloud.at(i).x = p.x;
    cloud.at(i).y = p.y;
    cloud.at(i).z = p.z;
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZ> Vga3DCamera::getEmptyPointCloud(int cols,
                                                               int rows) {
  pcl::PointXYZ point;
  point.x = std::numeric_limits<float>::quiet_NaN();
  point.y = std::numeric_limits<float>::quiet_NaN();
  point.z = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ> cloud(cols, rows, point);
  cloud.width = cols;
  cloud.height = rows;
  cloud.points.resize(cloud.width * cloud.height);
  cloud.is_dense = false;
  return cloud;
}

}  // namespace terabee
