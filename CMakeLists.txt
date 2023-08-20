cmake_minimum_required(VERSION 3.0.2)
project(confusion_ros VERSION 1.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wfatal-errors")

add_subdirectory(confusion)

include_directories(
  include
  confusion/include
  confusion/example
  test
)

MESSAGE(STATUS "Building ConFusion for ROS.")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  image_transport
  cv_bridge
  tf)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED) # TODO(Tim) OpenCV could be an optional dependency. Diagram.cpp and apriltag stuff are the only things that depend.

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(FILES
  TagMsg.msg
  TagArray.msg
  OdomWithState.msg)
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ExternalReferenceTrackingModuleRos
  CATKIN_DEPENDS roscpp image_transport cv_bridge message_runtime
  DEPENDS CERES OpenCV
)

add_library(ExternalReferenceTrackingModuleRos 
  src/external_reference_tracking_module_ros.cpp
  src/ros_conversions.cpp)
add_dependencies(ExternalReferenceTrackingModuleRos
  confusion_ros_generate_messages_cpp
  ${confusion_EXPORTED_TARGETS})

add_executable(visualize_pose 
  src/visualize_pose.cpp
  src/ros_conversions.cpp)
target_link_libraries(visualize_pose confusion ${catkin_LIBRARIES})

add_executable(tag_tracker_node 
  src/tag_tracker_node.cpp
  src/apriltag_module_ros.cpp
  src/ros_conversions.cpp)
target_link_libraries(tag_tracker_node confusion ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

catkin_add_gtest(parameterization_test 
        confusion/test/parameterization_test.cpp
        confusion/test/local_parameterization_test_fixture.cpp)
target_link_libraries(parameterization_test confusion ${external_libraries})

catkin_add_gtest(measurementManagerTest confusion/test/MeasurementManagerTest.cpp)
target_link_libraries(measurementManagerTest confusion ${external_libraries})

catkin_add_gtest(ImuChainTest confusion/test/ImuChainTest.cpp)
target_link_libraries(ImuChainTest confusion ${external_libraries})

catkin_add_gtest(PriorCostTest confusion/test/PriorCostTest.cpp)
target_link_libraries(PriorCostTest confusion ${external_libraries})

catkin_add_gtest(marginalizationTest confusion/test/marginalizationTest.cpp)
target_link_libraries(marginalizationTest confusion ${external_libraries})

catkin_add_gtest(distancesTest confusion/test/distancesTest.cpp)
target_link_libraries(distancesTest ${external_libraries})

catkin_add_gtest(rotationUtilsTest confusion/test/RotationUtilsTest.cpp)
target_link_libraries(rotationUtilsTest ${external_libraries})

catkin_add_gtest(static_parameter_random_walk_process_tester 
        confusion/test/static_parameter_random_walk_process_tester.cpp)
target_link_libraries(static_parameter_random_walk_process_tester confusion ${external_libraries})

catkin_add_gtest(tag_tracker_test test/tag_tracker_test.cpp)
target_link_libraries(tag_tracker_test confusion ${catkin_LIBRARIES} ${external_libraries})
# add_dependencies(tag_tracker_test confusion_generate_messages_cpp ${confusion_EXPORTED_TARGETS})









