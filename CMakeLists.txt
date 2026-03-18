cmake_minimum_required(VERSION 3.12)
project(rmcs_dart_guidance)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-Wall -Wextra -Wpedantic -O3)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(type_description_interfaces REQUIRED)

find_package(OpenCV 4.5 REQUIRED)
find_package(OpenVINO REQUIRED)
find_package(Ceres REQUIRED)

find_package(rosidl_typesupport_cpp REQUIRED)

find_package(rmcs_executor REQUIRED)
find_package(rmcs_description REQUIRED)
find_package(fast_tf REQUIRED)
find_package(hikcamera REQUIRED)

include_directories(
  ${PROJECT_SOURCE_DIR}/src/
  ${EIGEN3_INCLUDE_DIRS}
  ${type_description_interfaces_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmcs_executor_INCLUDE_DIRS}
  ${rmcs_description_INCLUDE_DIRS}
  ${fast_tf_INCLUDE_DIRS}
  ${hikcamera_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenVINO_INCLUDE_DIRS}
  ${Ceres_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

file(GLOB_RECURSE PROJECT_SOURCE
  CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/src/*.cpp
  ${PROJECT_SOURCE_DIR}/src/*.cc
)

file(GLOB DEBUG_COMPONENT_SOURCE
  CONFIGURE_DEPENDS
  ${PROJECT_SOURCE_DIR}/test/*.cpp
  ${PROJECT_SOURCE_DIR}/test/*.cc
)

add_library(
  ${PROJECT_NAME} SHARED
  ${PROJECT_SOURCE}
  ${DEBUG_COMPONENT_SOURCE}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  visualization_msgs
  rosidl_typesupport_cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${type_description_interfaces_LIBRARIES}
  
  ${OpenCV_LIBS}
  ${OpenVINO_LIBRARIES}
  ${Ceres_LIBRARIES}
  ${rmcs_executor_LIBRARIES}
  ${rmcs_description_LIBRARIES}
  ${fast_tf_LIBRARIES}
  ${hikcamera_LIBRARIES}
)

install(
  TARGETS 
    ${PROJECT_NAME}
  DESTINATION 
    lib/
)

find_package(pluginlib REQUIRED)
pluginlib_export_plugin_description_file(
  rmcs_executor plugins.xml
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    auto_aim_action_test
    tests/auto_aim_action_test.cpp
  )

  ament_add_gtest(
    launch_preparation_task_test
    tests/launch_preparation_task_test.cpp
  )

  ament_add_gtest(
    vision_assisted_launch_preparation_task_test
    tests/vision_assisted_launch_preparation_task_test.cpp
  )

  ament_add_gtest(
    dart_launch_sequence_test
    tests/dart_launch_sequence_test.cpp
  )

  foreach(target
      auto_aim_action_test
      launch_preparation_task_test
      vision_assisted_launch_preparation_task_test
      dart_launch_sequence_test)
    if(TARGET ${target})
      target_include_directories(${target} PRIVATE
        ${PROJECT_SOURCE_DIR}/src/
        ${EIGEN3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
      )

      ament_target_dependencies(${target}
        rclcpp
      )
    endif()
  endforeach()
endif()

ament_package()
