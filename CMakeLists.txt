cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_fusion)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
			roscpp
			message_generation)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  )
  
add_definitions(
  ${PCL_DEFINITIONS}
)
#catkin_package(CFG_EXTRAS export_flags.cmake)
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake/)
#include(${PROJECT_SOURCE_DIR}/cmake/export_flags.cmake)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)


# Main nodes
add_executable(point_cloud_fusion
			src/main_point_cloud_fusion.cpp
			src/PointCloudFusion.cpp)

target_link_libraries(point_cloud_fusion 
					${catkin_LIBRARIES}
					${PCL_LIBRARIES}
					${OpenCV_LIBS})

