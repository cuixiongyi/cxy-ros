cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
    cmake_policy(SET CMP0054 NEW)
endif()

project(perception_model_based_detection)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

find_package(PCL 1.7 REQUIRED common io visualization)
find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        geometry_msgs
                                        sensor_msgs
                                        pcl_conversions
                                        std_msgs
                                        pcl_ros
                                        )


include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(model_based_detection 
                src/lmicp.cpp
                src/cxy_transform.cpp
             )

target_link_libraries(model_based_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES model_based_detection
   CATKIN_DEPENDS roscpp pcl_conversions std_msgs   
   DEPENDS opencv pcl
)

add_executable(lmicp src/lmicp.cpp)
target_link_libraries(lmicp model_based_detection   ${catkin_LIBRARIES})
