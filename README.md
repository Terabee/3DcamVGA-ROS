# ROS package for the Terabee 3DcamVGA

## Supported hardware

This ROS package is supported and tested on:
* Ubuntu x86_64 18.04 with ROS Melodic,
* Ubuntu x86_64 20.04 with ROS Noetic.

This package works with the Terabee 3DcamVGA depth camera device.


## Dependencies

The 3DcamVGA software (contains a client API) must be installed in your system. This software is delivered with the 3DcamVGA device.


## Build

Clone this project in your workspace:
```
cd ~/ros_ws/src
git clone git@github.com:Terabee/3DcamVGA-ROS.git
```

Or, if you prefer to use https to clone::
```
cd ~/ros_ws/src
git clone https://github.com/Terabee/3DcamVGA-ROS.git
```

In your workspace, invoke catkin:
```
cd ~/ros_ws
catkin build
source devel/setup.bash
```


## ROS_VGA_3D_Camera node

To run the node:
```
rosrun vga_3d_camera camera_node _[param]:=[value]
```

Possible parameters:

* ip_address_ (string, default: "192.168.1.153") -> *ip address of the crozicam*

* port_ (integer, default: 8080) -> *camera control port*

* mode_ (string, default: custom19) -> *camera mode*

* id_frame_ (string, default: map) -> *frame id for point cloud*


## Published topics:

* /vga_3d_camera/image (sensor_msgs/Image)
* /vga_3d_camera/image/compressed (sensor_msgs/CompressedImage)
* /vga_3d_camera/image/compressed/parameter_descriptions (dynamic_reconfigure/ConfigDescription)
* /vga_3d_camera/image/compressed/parameter_updates (dynamic_reconfigure/Config)

* /vga_3d_camera/image/compressedDepth (sensor_msgs/CompressedImage)
* /vga_3d_camera/image/compressedDepth/parameter_descriptions (dynamic_reconfigure/ConfigDescription)
* /vga_3d_camera/image/compressedDepth/parameter_updates (dynamic_reconfigure/Config)

* /vga_3d_camera/image/theora (theora_image_transport/Packet)
* /vga_3d_camera/image/theora/parameter_descriptions (dynamic_reconfigure/ConfigDescription)
* /vga_3d_camera/image/theora/parameter_updates (dynamic_reconfigure/Config)

* /vga_3d_camera/parameter_descriptions (dynamic_reconfigure/ConfigDescription)
* /vga_3d_camera/parameter_updates (dynamic_reconfigure/Config)

* /vga_3d_camera/pointcloud (sensor_msgs/PointCloud2)


## Services

### /TempSrv

To run node for temperature service

```
rosrun vga_3d_camera temperature_service
```

Call the service with ip address and port as arguments
```
 rosservice call /TempSrv "ip_address_" port_
```

## Developers rules

On this repo we follow these rules:
* use the [GitFlow](https://nvie.com/posts/a-successful-git-branching-model/)
* apply [Google C++ style guide](https://google.github.io/styleguide/cppguide.html)
* use [Angular commit message format](https://github.com/angular/angular.js/blob/master/DEVELOPERS.md#commit-message-format)
