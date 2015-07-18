cmake_minimum_required(VERSION 2.8.3)
if(NOT ${CMAKE_VERSION} VERSION_LESS "3.1")
    cmake_policy(SET CMP0054 NEW)
endif()

project(project_lmicp)


set(CMAKE_CXX_FLAGS "-std=c++0x -g -march=native -mno-avx ${CMAKE_CXX_FLAGS} ${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall ")

option(ENABLE_MY_ASSERT "Turn on CXY_ASSERT checks" ON)

if(ENABLE_MY_ASSERT)
  add_definitions(-DENABLE_CXY_ASSERT)
else()
  add_definitions(-DMY_DEBUG=0)
endif()

find_package(OpenCV REQUIRED)

# find_package(Boost REQUIRED)
#find_package(Eigen3 REQUIRED)
set(${EIGEN3_INCLUDE_DIRS} "/usr/include/eigen3")
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




#include(/home/xiongyi/Downloads/lib/dlib-18.16/dlib/cmake)

# CMakeList for dlib
# ADD_EXECUTABLE(assignment_learning_ex assignment_learning_ex.cpp)
# TARGET_LINK_LIBRARIES(assignment_learning_ex dlib)

include_directories(include ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${Qt5Widgets_INCLUDES})


#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#

add_library(lmicp_lib 
                include/utility/cxy_transform.h src/utility/cxy_transform.cpp

                include/optimization/cxy_cost_func_abstract.h
                include/optimization/cxy_nonlinear_minimizer.h
                include/optimization/cxy_nonlinear_minimizer_LM.h

              include/energy/cxy_icp.h
              include/energy/cxy_icp_rigid.h
              include/energy/cxy_icp_rigid_func.h 		src/energy/cxy_icp_rigid_func.cpp 
              include/energy/cxy_icp_arti_ik.h  	src/energy/cxy_icp_arti_ik.cpp
              include/energy/cxy_icp_arti_ik_func.h  	src/energy/cxy_icp_arti_ik_func.cpp
              include/kinematic/cxy_icp_kinematic_node.h  src/kinematic/cxy_icp_kinematic_node.cpp
              include/kinematic/cxy_icp_kinematic_chain.h  src/kinematic/cxy_icp_kinematic_chain.cpp

)


target_link_libraries(lmicp_lib ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES}   )#dlib)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES lmicp_lib  
   CATKIN_DEPENDS roscpp std_msgs   
   DEPENDS opencv  
)

add_executable(icp_main src/app/icp_main.cpp)
add_executable(lmicp_exe src/app/lmicp.cpp)
#add_executable(convergence_test_main_exe src/app/convergence_test_main.cpp)
add_executable(main_exe src/app/main.cpp)
target_link_libraries(lmicp_exe ${catkin_LIBRARIES} lmicp_lib)
target_link_libraries(icp_main ${catkin_LIBRARIES} lmicp_lib)
#target_link_libraries(convergence_test_main_exe ${catkin_LIBRARIES} lmicp_lib)
target_link_libraries(main_exe ${catkin_LIBRARIES} lmicp_lib)
add_executable(main_new src/app/main_new.cpp)



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
