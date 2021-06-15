cmake_minimum_required(VERSION 3.5.1)
SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)
project(vga_3d_camera)

find_package(Terabee_3Dcam_VGA REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
  pcl_conversions
  message_generation
  std_msgs
)


generate_dynamic_reconfigure_options(
  cfg/Vga3DCamera.cfg
)

add_service_files(FILES
    TempSrv.srv
    )


generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp dynamic_reconfigure sensor_msgs dynamic_reconfigure image_transport cv_bridge message_runtime
  std_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CROZICAM_CLIENT_INCLUDE_DIRS}
)

add_executable(camera_node src/Vga3DCameraNode.cpp src/Vga3DCamera.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES})
target_link_libraries(camera_node ${CROZICAM_CLIENT_LIBRARIES})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(camera_node ${catkin_EXPORTED_TARGETS})
add_dependencies(camera_node ${PROJECT_NAME}_gencfg)

add_executable(temperature_service src/TemperatureService.cpp src/Vga3DCamera.cpp)
target_link_libraries(temperature_service ${catkin_LIBRARIES})
target_link_libraries(temperature_service ${CROZICAM_CLIENT_LIBRARIES})
target_include_directories(temperature_service PRIVATE ${catkin_INCLUDE_DIRS})

install( TARGETS
  camera_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_subdirectory(tests)
