# Standalone (not part of the colcon build):
#   cmake -S core/test -B <build> -DORB_SLAM3_DIR=<...>/core && cmake --build <build>
cmake_minimum_required(VERSION 3.8)
project(p9b_deterministic_tests)
set(CMAKE_CXX_STANDARD 17)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Pangolin REQUIRED)
set(ORB_SLAM3_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH "visual_sgraphs/core")
add_executable(deterministic_guard_test deterministic_guard_test.cc)
target_include_directories(deterministic_guard_test PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(deterministic_guard_test ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(deterministic_guard_test PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(test_pedestrian_factors test_pedestrian_factors.cc)
target_include_directories(test_pedestrian_factors PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(test_pedestrian_factors ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(test_pedestrian_factors PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(synthetic_stature synthetic_stature.cc)
target_include_directories(synthetic_stature PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(synthetic_stature ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(synthetic_stature PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(synthetic_floor_contact synthetic_floor_contact.cc)
target_include_directories(synthetic_floor_contact PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(synthetic_floor_contact ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(synthetic_floor_contact PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(synthetic_gate synthetic_gate.cc)
target_include_directories(synthetic_gate PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(synthetic_gate ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(synthetic_gate PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(synthetic_floor_px synthetic_floor_px.cc)
target_include_directories(synthetic_floor_px PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(synthetic_floor_px ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(synthetic_floor_px PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")

add_executable(synthetic_levels synthetic_levels.cc)
target_include_directories(synthetic_levels PRIVATE ${ORB_SLAM3_DIR} ${ORB_SLAM3_DIR}/include ${ORB_SLAM3_DIR}/include/CameraModels
  ${ORB_SLAM3_DIR}/Thirdparty/Sophus ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
target_link_libraries(synthetic_levels ${ORB_SLAM3_DIR}/lib/libvs_graphs_lib.so ${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
  ${ORB_SLAM3_DIR}/Thirdparty/g2o/lib/libg2o.so ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Pangolin_LIBRARIES})
set_target_properties(synthetic_levels PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${ORB_SLAM3_DIR}/lib;${ORB_SLAM3_DIR}/Thirdparty/DBoW2/lib;${ORB_SLAM3_DIR}/Thirdparty/g2o/lib")
