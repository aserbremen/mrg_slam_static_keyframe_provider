cmake_minimum_required(VERSION 3.8)
project(mrg_slam_map_provider)

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
add_library(map_provider_component SHARED
  src/MapProviderComponent.cpp
)
ament_target_dependencies(map_provider_component 
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
  visualization_msgs
  mrg_slam_msgs
  pcl_ros
)
rclcpp_components_register_nodes(map_provider_component "mrg_slam_map_provider::MapProviderComponent")

# Add the map_provider executable which is linked to the map_provider_component library
add_executable(map_provider_node src/MapProviderNode.cpp)
target_link_libraries(map_provider_node map_provider_component)

# Install the map_provider_component library
install(TARGETS
  map_provider_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the map_provider_node executable
install(TARGETS
  map_provider_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
