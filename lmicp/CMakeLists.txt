cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
    cmake_policy(SET CMP0054 NEW)
endif()

project(project_lmicp C CXX)


set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall ")

option(ENABLE_MY_ASSERT "Turn on CXY_ASSERT checks" ON)

if(ENABLE_MY_ASSERT)
  add_definitions(-DENABLE_CXY_ASSERT)
else()
  add_definitions(-DMY_DEBUG=0)
endif()
add_definitions(-DCXY_JACO_TYPE_FITTING)

#find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED)
#find_package(Eigen3 REQUIRED)
set(${EIGEN3_INCLUDE_DIRS} "/usr/include/eigen3")
find_package(PCL 1.7 REQUIRED common io visualization)
find_package(Boost REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp
                                        rospy
                                        roslib
                                        std_msgs
                                        geometry_msgs
                                        sensor_msgs
                                        shape_msgs
                                        geometric_shapes
                                        pcl_ros
                                        )




#include(/home/xiongyi/Downloads/lib/dlib-18.16/dlib/cmake)

# CMakeList for dlib
# ADD_EXECUTABLE(assignment_learning_ex assignment_learning_ex.cpp)
# TARGET_LINK_LIBRARIES(assignment_learning_ex dlib)

include_directories(include ${PCL_INCLUDE_DIRS}  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Qt5Widgets_INCLUDES})
#${OpenCV_INCLUDE_DIRS}

#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#

add_library(cxy_tracking_lib

                include/common/cxy_config.h                     src/common/cxy_config.cpp

                include/utility/cxy_transform.h                 src/utility/cxy_transform.cpp
                include/utility/cxy_CAD_helper.h                 src/utility/cxy_CAD_helper.cpp
                include/utility/cxy_sync.h                      src/utility/cxy_sync.cpp
                include/utility/cxy_modelCloud_engin.h          src/utility/cxy_modelCloud_engin.cpp
                include/utility/cxy_publisher.h                 src/utility/cxy_publisher.cpp
#                include/common/hack.hpp

                include/optimization/cxy_cost_func_abstract.h
                include/optimization/cxy_nonlinear_minimizer.h
                include/optimization/cxy_nonlinear_minimizer_LM.h

#              include/energy/cxy_icp.h
#              include/energy/cxy_icp_rigid.h
#              include/energy/cxy_icp_rigid_func.h 		        src/energy/cxy_icp_rigid_func.cpp
#              include/energy/cxy_icp_arti_ik.h  	            src/energy/cxy_icp_arti_ik.cpp
#              include/energy/cxy_icp_arti_ik_func.h  	        src/energy/cxy_icp_arti_ik_func.cpp
#              include/energy/cxy_energy_fitting.h               src/energy/cxy_energy_fitting.cpp
              include/kinematic/cxy_icp_kinematic.h             src/kinematic/cxy_icp_kinematic.cpp
              include/kinematic/cxy_icp_kinematic_joint.h       src/kinematic/cxy_icp_kinematic_joint.cpp
              include/kinematic/cxy_icp_kinematic_chain.h       src/kinematic/cxy_icp_kinematic_chain.cpp
              include/kinematic/cxy_icp_kinematic_point.h       src/kinematic/cxy_icp_kinematic_point.cpp
              include/tracker/cxy_tracker.h       src/tracker/cxy_tracker.cpp
)




catkin_package(
   INCLUDE_DIRS include
   LIBRARIES cxy_tracking_lib
   CATKIN_DEPENDS roscpp std_msgs   geometry_msgs sensor_msgs shape_msgs geometric_shapes
   #DEPENDS opencv
)

target_link_libraries(cxy_tracking_lib  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})

#add_executable(icp_main src/app/icp_main.cpp)
#add_executable(lmicp_exe src/app/lmicp.cpp)
#add_executable(main_exe src/app/main.cpp)
#target_link_libraries(lmicp_exe ${catkin_LIBRARIES} cxy_tracking_lib)
#target_link_libraries(icp_main ${catkin_LIBRARIES} cxy_tracking_lib)
#target_link_libraries(main_exe ${catkin_LIBRARIES} cxy_tracking_lib)
add_executable(transform_test src/app/transform_test.cpp)
target_link_libraries(transform_test ${catkin_LIBRARIES} cxy_tracking_lib)
add_executable(new_icp src/app/new_icp.cpp)
target_link_libraries(new_icp ${catkin_LIBRARIES} cxy_tracking_lib)



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
# Qt5
set (CMAKE_PREFIX_PATH "/opt/Qt/5.5/gcc_64")
# Find the QtWidgets library
find_package(Qt5Widgets REQUIRED)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTOUIC ON)
#add_definitions(${Qt5Widgets_DEFINITIONS})

set (MyProjectLib_src ${PROJECT_SOURCE_DIR}/src/gui/mainwindow.cpp)
set (MyProjectLib_hdr ${PROJECT_SOURCE_DIR}/include/gui/mainwindow.h)
set (MyProjectLib_ui  ${PROJECT_SOURCE_DIR}/include/gui/mainwindow.ui)
set (MyProjectBin_src ${PROJECT_SOURCE_DIR}/src/gui/main.cpp)

qt5_wrap_cpp(MyProjectLib_hdr_moc ${MyProjectLib_hdr})
qt5_wrap_ui (MyProjectLib_ui_moc  ${MyProjectLib_ui})

include_directories (${PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})

add_library (ygui_lib SHARED 
            ${MyProjectLib_src}
            ${MyProjectLib_hdr_moc}
            ${MyProjectLib_ui_moc}
)
target_link_libraries(ygui_lib Qt5::Widgets) #${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES}  


add_executable(ygui ${MyProjectBin_src} )
target_link_libraries(ygui ygui_lib  )
qt5_use_modules(ygui_lib Core Gui Widgets)
