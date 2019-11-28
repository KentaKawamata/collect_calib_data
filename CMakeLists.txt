cmake_minimum_required(VERSION 2.8.3)
project(collect_calib_data)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  tf2_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS geometry_msgs pcl_conversions pcl_ros roscpp rospy std_msgs tf2_ros
    DEPENDS
)

include_directories(
   include
   ${catkin_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   ${Eigen3_INCLUDE_DIRS}
)

add_executable(superimpose_velo_points
   src/make_pc.cpp
   include/make_pc.hpp
   src/editCloud.cpp
   include/editCloud.hpp
   src/getRotationVector.cpp
   include/getRotationVector.hpp
)

add_dependencies(calib_velodyne
   ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(superimpose_velo_points
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${Eigen3_LIBRARIES}
)
