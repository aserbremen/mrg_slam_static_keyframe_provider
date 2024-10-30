#include "MapProviderComponent.cpp"

int
main( int argc, char *argv[] )
{
    // This main will be linked against the static_map_provider_component library
    rclcpp::init( argc, argv );
    rclcpp::executors::MultiThreadedExecutor executor( rclcpp::ExecutorOptions(), 4 );

    rclcpp::NodeOptions options;
    auto                map_provider_node = std::make_shared<mrg_slam_map_provider::MapProviderComponent>( options );

    executor.add_node( map_provider_node );
    executor.spin();
    rclcpp::shutdown();
    return 0;
}