cmake_minimum_required(VERSION 3.5)  # ros2 change
project(vs_graphs)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3")
# [Hint] We commented below lines as it got stuck in the normalize function of g2o::plane3d
# set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
# set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # Check C++14 or C++0x support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

# if(COMPILER_SUPPORTS_CXX14)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#   add_definitions(-DCOMPILEDWITHC11)
#   message(STATUS "Using flag -std=c++14.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#   add_definitions(-DCOMPILEDWITHC0X)
#   message(STATUS "Using flag -std=c++0x.")
# else()
#   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
# endif()


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.2)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()

MESSAGE("OPENCV VERSION:")
MESSAGE(${OpenCV_VERSION})

## Find ament macros and libraries
## Use 'find_package' for ROS 2 packages instead of 'catkin'
## Find ament and ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
# find_package(aruco_ros REQUIRED)  # Comment out - check if ROS2 equivalent exists
# find_package(aruco_msgs REQUIRED)  # Comment out - check if ROS2 equivalent exists
find_package(backward_ros REQUIRED)  # Comment out - check if ROS2 equivalent exists
find_package(rviz_visual_tools REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(segmenter_ros REQUIRED)  # Comment out - check if ROS2 equivalent exists


find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
# find_package(PCL REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  "msg/VSGraphsWallData.msg"
  "msg/VSGraphsRoomData.msg"
  "msg/VSGraphsAllWallsData.msg"
  "msg/VSGraphsAllDetectdetRooms.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/core
  ${PROJECT_SOURCE_DIR}/core/include
  ${PROJECT_SOURCE_DIR}/core/include/CameraModels
  ${PROJECT_SOURCE_DIR}/core/Thirdparty
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${segmenter_ros_INCLUDE_DIRS}
)
include_directories(${OPENGL_INCLUDE_DIRS})

# ROS2 uses ament_target_dependencies instead of catkin_INCLUDE_DIRS
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include third-party CMakeLists
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/CMakeLists.txt)

# Set library output directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/core/lib)

# Create the main library
add_library(${PROJECT_NAME}_lib SHARED
  core/src/System.cc
  core/src/Tracking.cc
  core/src/LocalMapping.cc
  core/src/LoopClosing.cc
  core/src/ORBextractor.cc
  core/src/ORBmatcher.cc
  core/src/FrameDrawer.cc
  core/src/Converter.cc
  core/src/MapPoint.cc
  core/src/KeyFrame.cc
  core/src/Atlas.cc
  core/src/Map.cc
  core/src/MapDrawer.cc
  core/src/Optimizer.cc
  core/src/Frame.cc
  core/src/KeyFrameDatabase.cc
  core/src/Sim3Solver.cc
  core/src/Viewer.cc
  core/src/ImuTypes.cc
  core/src/G2oTypes.cc
  core/src/CameraModels/Pinhole.cpp
  core/src/CameraModels/KannalaBrandt8.cpp
  core/src/OptimizableTypes.cpp
  core/src/MLPnPsolver.cpp
  core/src/GeometricTools.cc
  core/src/TwoViewReconstruction.cc
  core/src/Utils.cc
  core/src/Config.cc
  core/src/Settings.cc
  core/src/GeoSemHelpers.cc
  core/src/Geometric/Plane.cc
  core/src/Semantic/Marker.cc
  core/src/Semantic/Door.cc
  core/src/Semantic/Room.cc
  core/src/Semantic/Floor.cc
  core/src/DatabaseParser.cc
  core/src/Types/SystemParams.cc
  core/src/GeometricSegmentation.cc
  core/src/SemanticSegmentation.cc
  core/src/SemanticsManager.cc
  core/include/System.h
  core/include/Tracking.h
  core/include/LocalMapping.h
  core/include/LoopClosing.h
  core/include/ORBextractor.h
  core/include/ORBmatcher.h
  core/include/FrameDrawer.h
  core/include/Converter.h
  core/include/MapPoint.h
  core/include/KeyFrame.h
  core/include/Atlas.h
  core/include/Map.h
  core/include/MapDrawer.h
  core/include/Optimizer.h
  core/include/Frame.h
  core/include/KeyFrameDatabase.h
  core/include/Sim3Solver.h
  core/include/Viewer.h
  core/include/ImuTypes.h
  core/include/G2oTypes.h
  core/include/CameraModels/GeometricCamera.h
  core/include/CameraModels/Pinhole.h
  core/include/CameraModels/KannalaBrandt8.h
  core/include/OptimizableTypes.h
  core/include/MLPnPsolver.h
  core/include/GeometricTools.h
  core/include/TwoViewReconstruction.h
  core/include/SerializationUtils.h
  core/include/Config.h
  core/include/Utils.h
  core/include/Settings.h
  core/include/GeoSemHelpers.h
  core/include/Semantic/Door.h
  core/include/Semantic/Room.h
  core/include/Semantic/Floor.h
  core/include/DatabaseParser.h
  core/include/Semantic/Marker.h
  core/include/Geometric/Plane.h
  core/include/Types/SystemParams.h
  core/include/GeometricSegmentation.h
  core/include/SemanticSegmentation.h
  core/include/SemanticsManager.h
)

target_link_libraries(${PROJECT_NAME}_lib 
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/core/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# ROS2 specific: Use ament_target_dependencies for ROS2 packages
ament_target_dependencies(${PROJECT_NAME}_lib 
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav_msgs
  message_filters
  image_transport
  pcl_ros
  rviz_visual_tools
  # segmenter_ros
)


## ROS2 node
add_executable(ros_rgbd
  src/ros_rgbd.cc
  src/common.cc
)
target_link_libraries(ros_rgbd
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(ros_rgbd
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  std_msgs
  nav_msgs
  message_filters
  tf2
  tf2_ros
  segmenter_ros
)

## ROS2 node
# add_executable(ros_rgbd_inertial
#   src/ros_rgbd_inertial.cc
#   src/common.cc
# )

# target_link_libraries(ros_rgbd_inertial
#   ${PROJECT_NAME}_lib
# )
# ament_target_dependencies(ros_rgbd_inertial
#   rclcpp
#   sensor_msgs
#   geometry_msgs
#   cv_bridge
#   std_msgs
#   nav_msgs
#   message_filters
#   tf2
#   tf2_ros
#   segmenter_ros
# )

# Make sure executables can use the custom messages
rosidl_target_interfaces(ros_rgbd ${PROJECT_NAME} "rosidl_typesupport_cpp")
# rosidl_target_interfaces(ros_rgbd_inertial ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(
  DIRECTORY launch config core/Vocabulary
  DESTINATION share/${PROJECT_NAME}
)

# Install the library
install(TARGETS ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  ros_rgbd
  # ros_rgbd_inertial
  DESTINATION lib/${PROJECT_NAME}
)

# IMPORTANT: This must be the very last line in CMakeLists.txt
ament_package()