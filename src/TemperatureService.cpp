#include <ros/ros.h>
#include <CameraClient/ClientFactory.hpp>
#include <CameraClient/ICameraClient.hpp>
#include <vga_3d_camera/TempSrv.h>

#include "Vga3DCamera/Vga3DCamera.hpp"

bool temperature(vga_3d_camera::TempSrv::Request &req,
                 vga_3d_camera::TempSrv::Response &res) {
  std::pair<float, float> temp;
  std::shared_ptr<terabee::cameraclient::ICameraClient> client_;
  std::string ip_address = req.ip;
  int port = req.port;
  client_ = terabee::cameraclient::ClientFactory::createCameraClient(ip_address,
                                                                     port);
  client_->connect();
  if (!client_->validateConnection()) {
    ROS_INFO("Failed to validate connection");
    return false;
  }
  temp = client_->getCameraTemperatures();
  ROS_INFO("Gettin temp");
  float temp_AFE = temp.first;
  float temp_Laser = temp.second;
  res.result_AFE = temp_AFE;
  res.result_Laser = temp_Laser;
  client_->disconnect();
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "temperature_service");
  ros::NodeHandle nh;
  ROS_INFO("Connected");
  terabee::Vga3DCamera camera;
  ros::ServiceServer service = nh.advertiseService("TempSrv", temperature);
  ROS_INFO("Ready to get temperature.");
  ros::spin();
  return 0;
}
