cmake_minimum_required(VERSION 3.8)
project(mrg_slam_static_keyframe_provider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mrg_slam_msgs REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(include)

# map_provider component
add_library(static_keyframe_provider_component SHARED
  src/StaticKeyFrameProviderComponent.cpp
)
ament_target_dependencies(static_keyframe_provider_component 
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  visualization_msgs
  mrg_slam_msgs
  pcl_ros
)
rclcpp_components_register_nodes(static_keyframe_provider_component "mrg_slam::StaticKeyFrameProviderComponent")

# Add the map_provider executable which is linked to the static_keyframe_provider_component library
add_executable(static_keyframe_provider_node src/StaticKeyFrameProviderNode.cpp)
target_link_libraries(static_keyframe_provider_node static_keyframe_provider_component)

# Install the static_keyframe_provider_component library
install(TARGETS
  static_keyframe_provider_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the static_keyframe_provider_node executable
install(TARGETS
  static_keyframe_provider_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
