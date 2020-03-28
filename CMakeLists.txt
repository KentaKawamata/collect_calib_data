cmake_minimum_required(VERSION 2.8.3)
project(collect_calib_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  tf2_ros
  genmsg
  actionlib
  actionlib_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# アクションメッセージの定義
#####################################
add_action_files(
    DIRECTORY action
    FILES calib_velo.action
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)
#####################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS  roscpp
                    rospy
                    geometry_msgs
                    sensor_msgs
                    std_msgs
                    pcl_conversions
                    pcl_ros
                    tf2_ros 
                    actionlib
                    actionlib_msgs
    DEPENDS
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   ${Eigen3_INCLUDE_DIRS}
)


#####################################
add_executable(calib_velodyne
   src/calib_velodyne.cpp
   include/calib_velodyne.h
)

add_dependencies(calib_velodyne
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(calib_velodyne
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
#####################################


#####################################
add_executable(calib_ts_server
   src/calib_ts_server.cpp
   include/calib_ts_server.h
)

add_dependencies(calib_ts_server
   ${catkin_EXPORTED_TARGETS}
   ${ts_server_EXPORTED_TARGETS}
)

target_link_libraries(calib_ts_server
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
#####################################
