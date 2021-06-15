#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>

#include <CameraClient/ClientFactory.hpp>
#include <CameraClient/ICameraClient.hpp>

#include "Vga3DCamera/Vga3DCamera.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "vga_3d_camera");

  terabee::Vga3DCamera camera;

  camera.spin();

  return 0;
}
