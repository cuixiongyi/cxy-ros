cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
    cmake_policy(SET CMP0054 NEW)
endif()

project(plastic_detection)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        )


include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#
#add_library( plastic_detection_lib
#                src/lmicp.cpp
#                src/cxy_transform.cpp
#             )

#target_link_libraries(plastic_detection_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES plastic_detection
   CATKIN_DEPENDS roscpp std_msgs   
   DEPENDS opencv 
)

add_executable(plastic_exe src/main.cpp)
target_link_libraries(plastic_exe  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
