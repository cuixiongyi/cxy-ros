cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
    cmake_policy(SET CMP0054 NEW)
endif()

project(project_lmicp)

set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS}")

find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED)
find_package(VXL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED common io visualization)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        geometry_msgs
                                        sensor_msgs
                                        std_msgs
                                        pcl_ros
                                        )



include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#
add_library(lmicp_lib 
                src/cxy_transform.cpp
             )

           target_link_libraries(lmicp_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${VXL_LIBRARIES} ${EIGEN3_LIBRARIES})


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES lmicp_lib  
   CATKIN_DEPENDS roscpp std_msgs   
   DEPENDS opencv eigen3 
)

add_executable(lmicp_exe src/lmicp.cpp)
target_link_libraries(lmicp_exe ${catkin_LIBRARIES} lmicp_lib) 
