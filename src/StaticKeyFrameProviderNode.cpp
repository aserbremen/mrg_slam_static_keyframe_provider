#include "StaticKeyFrameProviderComponent.cpp"

int
main( int argc, char *argv[] )
{
    // This main will be linked against the static_map_provider_component library
    rclcpp::init( argc, argv );
    rclcpp::executors::MultiThreadedExecutor executor( rclcpp::ExecutorOptions(), 4 );

    rclcpp::NodeOptions options;
    auto                static_keyframe_provider_node = std::make_shared<mrg_slam::StaticKeyFrameProviderComponent>( options );

    executor.add_node( static_keyframe_provider_node );
    executor.spin();
    rclcpp::shutdown();
    return 0;
}