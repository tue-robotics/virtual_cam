cmake_minimum_required(VERSION 2.8.3)
project(virtual_cam)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation rosbag cv_bridge tf)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate services in the 'srv' folder
add_service_files(
  FILES
  cheese.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES virtual_cam_image_loader
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# Declare a cpp library
add_library(virtual_cam_image_loader
    src/Loader.cpp
    src/Image.cpp
)
target_link_libraries(virtual_cam_image_loader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(save src/save.cpp)
target_link_libraries(save yaml-cpp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(load src/load.cpp)
target_link_libraries(load yaml-cpp virtual_cam_image_loader)

add_executable(view src/view.cpp)
target_link_libraries(view virtual_cam_image_loader)

add_dependencies(save ${PROJECT_NAME}_gencpp)
