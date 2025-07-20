// pcl
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// boost
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/uuid_io.hpp>
// ROS2
#include <ament_index_cpp/get_package_share_directory.hpp>
// mrg_slam_static_keyframe_provider
#include <mrg_slam_static_keyframe_provider/StaticKeyFrameProviderComponent.hpp>
//
#include <openssl/sha.h>
#include <uuid/uuid.h>

#include <iomanip>
#include <sstream>

namespace mrg_slam {

// Define your own namespace UUID
const uuid_t UUID_NAMESPACE = { 0x6b, 0xa7, 0xb8, 0x10, 0x9d, 0xad, 0x11, 0xd1, 0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8 };


StaticKeyFrameProviderComponent::StaticKeyFrameProviderComponent( const rclcpp::NodeOptions &options ) :
    Node( "static_keyframe_provider", options ), patch_index( 0 )
{
    RCLCPP_INFO( get_logger(), "StaticKeyFrameProviderComponent has been started." );

    frame_id        = declare_parameter( "frame_id", "map" );
    grid_step_size  = static_cast<float>( declare_parameter( "grid_step_size", 15.0 ) );
    keyframe_radius = static_cast<float>( declare_parameter( "keyframe_radius", -1.0 ) );
    if( keyframe_radius < 0 ) {
        // Set the keyframe radius to the diagonal of the grid cell
        keyframe_radius = static_cast<float>( grid_step_size / std::sqrt( 2.0 ) );
    }
    pcd_path                 = declare_parameter( "pcd_path", "/path/to/pointcloud.pcd" );
    timer_frequency          = declare_parameter( "timer_frequency", 3.0 );
    robot_names              = declare_parameter( "robot_names", std::vector<std::string>{ "atlas", "bestla" } );
    slam_distance            = static_cast<float>( declare_parameter( "slam_distance", 30.0 ) );
    enable_voxel_grid_filter = declare_parameter( "enable_voxel_grid_filter", true );
    voxel_grid_resolution    = static_cast<float>( declare_parameter( "voxel_grid_resolution", 0.1 ) );

    squared_slam_distance = slam_distance * slam_distance;

    // Callback groups
    // for slam_pose_broadcast_sub + mrg_slam service clients
    reentrant_callback_group1 = create_callback_group( rclcpp::CallbackGroupType::Reentrant );
    // for keyframes_pub_timer + keyframes_pub + map_pub + marker_pub
    reentrant_callback_group2 = create_callback_group( rclcpp::CallbackGroupType::Reentrant );
    rclcpp::PublisherOptions pub_options;
    pub_options.callback_group = reentrant_callback_group2;


    // Publishers
    keyframes_pub = create_publisher<sensor_msgs::msg::PointCloud2>( "/static_keyframe_provider/keyframes", 10, pub_options );
    map_pub       = create_publisher<sensor_msgs::msg::PointCloud2>( "/static_keyframe_provider/map", 10, pub_options );
    marker_pub    = create_publisher<visualization_msgs::msg::MarkerArray>( "/static_keyframe_provider/markers", 10, pub_options );

    // Service clients
    for( const auto &robot_name : robot_names ) {
        std::string get_graph_uuids_service_name = "/" + robot_name + "/mrg_slam/get_graph_uuids";
        get_graph_uuids_clients[robot_name]      = create_client<mrg_slam_msgs::srv::GetGraphUuids>( get_graph_uuids_service_name,
                                                                                                     rmw_qos_profile_services_default,
                                                                                                     reentrant_callback_group1 );

        std::string add_static_keyframes_service_name = "/" + robot_name + "/mrg_slam/add_static_keyframes";
        add_static_keyframes_clients[robot_name] = create_client<mrg_slam_msgs::srv::AddStaticKeyFrames>( add_static_keyframes_service_name,
                                                                                                          rmw_qos_profile_services_default,
                                                                                                          reentrant_callback_group1 );
    }

    // Service servers, this is mainly used for testing purposes
    publish_map_service_server       = create_service<std_srvs::srv::Trigger>( "/static_keyframe_provider/publish_map",
                                                                               std::bind( &StaticKeyFrameProviderComponent::publish_map_service,
                                                                                          this, std::placeholders::_1, std::placeholders::_2 ) );
    publish_keyframes_service_server = create_service<std_srvs::srv::Trigger>(
        "/static_keyframe_provider/publish_keyframes",
        std::bind( &StaticKeyFrameProviderComponent::publish_keyframes_service, this, std::placeholders::_1, std::placeholders::_2 ) );

    create_static_keyframes();

    // Define the timer for publishing patches, start it only with the publish_static_map service call
    keyframes_pub_timer = create_wall_timer( std::chrono::milliseconds( static_cast<int64_t>( 1000 / timer_frequency ) ),
                                             std::bind( &StaticKeyFrameProviderComponent::patch_pub_timer_callback, this ),
                                             reentrant_callback_group2 );
    keyframes_pub_timer->cancel();
}

void
StaticKeyFrameProviderComponent::create_static_keyframes()
{
    pcl::PointCloud<PointT>::Ptr cloud = load_pcd( pcd_path );

    if( enable_voxel_grid_filter ) {
        // Apply voxel grid filter to downsample the point cloud
        pcl::VoxelGrid<PointT> voxel_grid;
        voxel_grid.setInputCloud( cloud );
        voxel_grid.setLeafSize( voxel_grid_resolution, voxel_grid_resolution, voxel_grid_resolution );
        pcl::PointCloud<PointT>::Ptr filtered_cloud( new pcl::PointCloud<PointT> );
        voxel_grid.filter( *filtered_cloud );
        cloud = filtered_cloud;
        RCLCPP_INFO_STREAM( get_logger(), "Applied voxel grid filter with resolution: " << voxel_grid_resolution << ", resulting in "
                                                                                        << cloud->size() << " points" );
    }

    // create a cloud with z = 0.0
    pcl::PointCloud<PointT>::Ptr cloud_xy( new pcl::PointCloud<PointT> );
    for( const auto &point : cloud->points ) {
        PointT temp_point = point;
        temp_point.z      = 0;
        cloud_xy->points.push_back( temp_point );
    }

    // Initialize KdTree for fast nearest neighbor search
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud( cloud_xy );

    // Determine the x and y bounds of the point cloud
    PointT min_pt, max_pt;
    pcl::getMinMax3D( *cloud_xy, min_pt, max_pt );

    // Generate grid centers based on x-y plane and grid_step_size
    for( float x = min_pt.x; x <= max_pt.x; x += grid_step_size ) {
        for( float y = min_pt.y; y <= max_pt.y; y += grid_step_size ) {
            StaticKeyframe::Ptr static_keyframe = std::make_shared<StaticKeyframe>();

            PointT center( x, y, 0 );  // Grid center for current patch

            // Search for points within the keyframe_radius from the current grid center
            std::vector<int>   point_indices;
            std::vector<float> point_distances;

            if( kdtree.radiusSearch( center, keyframe_radius, point_indices, point_distances ) > 0 ) {
                // Create a new cloud for the patch
                pcl::PointCloud<PointT>::Ptr patch( new pcl::PointCloud<PointT> );

                // Extract points within the radius
                for( int idx : point_indices ) {
                    // push_back the original point and not the one with z = 0
                    patch->points.push_back( cloud->points[idx] );
                }

                // Calculate the center point where the z-value is the average of the z-values of the points in the patch
                float z_sum = 0;
                for( const auto &point : patch->points ) {
                    z_sum += point.z;
                }
                center.z                = z_sum / patch->size();
                static_keyframe->center = center;

                // Transform the patch points to the center, as if it was recorded at the center point
                for( auto &point : patch->points ) {
                    point.x -= center.x;
                    point.y -= center.y;
                    point.z -= center.z;
                }

                // Generate a reproducable UUID from the deterministic keyframe information
                static_keyframe->uuid_str = uuid_str_from_keyframe( x, y, patch->size() );

                RCLCPP_INFO_STREAM( get_logger(), "Created kf " << static_keyframe->uuid_str << " at center (" << x << ", " << y
                                                                << ") with " << patch->size() << " points" );
                static_keyframe->patch = patch;

                sensor_msgs::msg::PointCloud2::SharedPtr patch_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
                pcl::toROSMsg( *patch, *patch_msg );
                patch_msg->header.frame_id = "keyframe_point";  // cannot be visualized in rviz, for later usage in mrg_slam
                patch_msg->header.stamp    = now();
                static_keyframe->patch_msg = patch_msg;

                static_keyframes.push_back( static_keyframe );
            }
        }
    }

    // Subscribers
    // Use a reentrant callbackgroup for odom_broadcast_sub to avoid deadlock, enabling the get graph gids service to be called from the
    // same thread as the slam_pose_broadcast_callback
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = reentrant_callback_group1;
    slam_pose_broadcast_sub    = create_subscription<mrg_slam_msgs::msg::PoseWithName>(
        "/mrg_slam/slam_pose_broadcast", rclcpp::QoS( 100 ),
        std::bind( &StaticKeyFrameProviderComponent::slam_pose_broadcast_callback, this, std::placeholders::_1 ), sub_options );
    RCLCPP_INFO_STREAM( get_logger(), "Created slam pose_broadcast_sub" );
}

pcl::PointCloud<StaticKeyFrameProviderComponent::PointT>::Ptr
StaticKeyFrameProviderComponent::load_pcd( const std::string &pcd_path )
{
    pcl::PointCloud<PointT>::Ptr cloud( new pcl::PointCloud<PointT> );

    bool        path_is_abs  = pcd_path[0] == '/';
    std::string path_to_open = path_is_abs
                                   ? pcd_path
                                   : ament_index_cpp::get_package_share_directory( "mrg_slam_static_keyframe_provider" ) + "/" + pcd_path;

    RCLCPP_INFO_STREAM( get_logger(), "Loading point cloud from " << path_to_open );

    if( pcl::io::loadPCDFile<PointT>( path_to_open, *cloud ) == -1 ) {
        RCLCPP_ERROR( get_logger(), "Couldn't read file %s\n", path_to_open.c_str() );
        exit( EXIT_FAILURE );
        return nullptr;
    }
    RCLCPP_INFO_STREAM( get_logger(), "Loaded " << cloud->size() << " data points from " << path_to_open );

    if( !full_map ) {
        full_map = std::make_shared<sensor_msgs::msg::PointCloud2>();
    }
    pcl::toROSMsg( *cloud, *full_map );
    full_map->header.frame_id = frame_id;
    full_map->header.stamp    = now();

    return cloud;
}

std::string
StaticKeyFrameProviderComponent::uuid_str_from_keyframe( double x, double y, size_t num_points )
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision( 6 ) << x << "," << y << "," << num_points;
    std::string deterministic_uuid_str = ss.str();

    // Compute SHA-1 hash of the grid center string
    unsigned char hash[SHA_DIGEST_LENGTH];
    SHA1( reinterpret_cast<const unsigned char *>( deterministic_uuid_str.c_str() ), deterministic_uuid_str.size(), hash );

    // Convert the hash to a UUID
    uuid_t uuid;
    uuid_generate_md5( uuid, UUID_NAMESPACE, reinterpret_cast<const char *>( hash ), SHA_DIGEST_LENGTH );

    // Convert UUID to string
    char uuid_str[37];
    uuid_unparse( uuid, uuid_str );

    return std::string( uuid_str );
}

void
StaticKeyFrameProviderComponent::slam_pose_broadcast_callback( mrg_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg )
{
    mrg_slam_msgs::srv::GetGraphUuids::Response::SharedPtr gids   = get_graph_gids_service_call( slam_pose_msg->robot_name );
    auto                                                   logger = rclcpp::get_logger( "slam_pose_broadcast_callback" );
    if( !gids ) {
        RCLCPP_ERROR_STREAM( logger, "Failed to get graph gids for robot " << slam_pose_msg->robot_name );
        return;
    }
    if( gids->keyframe_uuid_strs.empty() ) {
        RCLCPP_INFO_STREAM( logger, "No keyframes in graph for robot " << slam_pose_msg->robot_name );
        return;
    }

    std::vector<StaticKeyframe::Ptr> static_keyframes_to_add;
    PointT slam_pose = PointT( static_cast<float>( slam_pose_msg->pose.position.x ), static_cast<float>( slam_pose_msg->pose.position.y ),
                               static_cast<float>( slam_pose_msg->pose.position.z ) );
    for( const auto &kf : static_keyframes ) {
        // Add the static keyframes that are not in the graph and are within the set slam distance
        if( std::find( gids->keyframe_uuid_strs.begin(), gids->keyframe_uuid_strs.end(), kf->uuid_str ) == gids->keyframe_uuid_strs.end()
            && ( slam_pose.getVector3fMap() - kf->center.getVector3fMap() ).head( 2 ).squaredNorm() < squared_slam_distance ) {
            kf->patch_msg->header.frame_id = slam_pose_msg->robot_name + "/map";
            static_keyframes_to_add.push_back( kf );
        }
    }

    if( static_keyframes_to_add.empty() ) {
        RCLCPP_INFO_STREAM( logger, "No unknown static keyframes in slam distance for robot " << slam_pose_msg->robot_name );
        return;
    }

    if( add_static_keyframes_service_call( slam_pose_msg, static_keyframes_to_add ) ) {
        RCLCPP_INFO_STREAM( logger, "Successfully added static map to robot " << slam_pose_msg->robot_name );
    } else {
        RCLCPP_ERROR_STREAM( logger, "Failed to add static map to robot " << slam_pose_msg->robot_name );
    }
}

mrg_slam_msgs::srv::GetGraphUuids::Response::SharedPtr
StaticKeyFrameProviderComponent::get_graph_gids_service_call( const std::string &robot_name )
{
    // Check if the service is available
    while( !get_graph_uuids_clients[robot_name]->wait_for_service( std::chrono::seconds( 2 ) ) ) {
        if( !rclcpp::ok() ) {
            return nullptr;
        }
        RCLCPP_WARN_STREAM_THROTTLE( get_logger(), *get_clock(), 1000,
                                     "Waiting for service " << get_graph_uuids_clients[robot_name]->get_service_name() << " to appear..." );
    }

    RCLCPP_INFO_STREAM( get_logger(), "Requesting graph gids for robot " << robot_name );
    mrg_slam_msgs::srv::GetGraphUuids::Request::SharedPtr req = std::make_shared<mrg_slam_msgs::srv::GetGraphUuids::Request>();

    auto               result_future = get_graph_uuids_clients[robot_name]->async_send_request( req );
    std::future_status status        = result_future.wait_for( std::chrono::seconds( 1 ) );

    if( status == std::future_status::timeout ) {
        RCLCPP_WARN_STREAM( get_logger(), "Request graph gids service call to robot " << robot_name << " timed out" );
        return nullptr;
    }
    if( status == std::future_status::ready ) {
        RCLCPP_INFO_STREAM( get_logger(), "Request graph gids service call to robot " << robot_name << " successful" );
    }
    return result_future.get();
}

bool
StaticKeyFrameProviderComponent::add_static_keyframes_service_call( mrg_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg,
                                                                    const std::vector<StaticKeyframe::Ptr> &static_keyframes_to_add )
{
    // Check if the service is available
    while( !add_static_keyframes_clients[slam_pose_msg->robot_name]->wait_for_service( std::chrono::seconds( 2 ) ) ) {
        if( !rclcpp::ok() ) {
            return false;
        }
        RCLCPP_WARN_STREAM_THROTTLE( get_logger(), *get_clock(), 1000,
                                     "Waiting for service " << add_static_keyframes_clients[slam_pose_msg->robot_name]->get_service_name()
                                                            << " to appear..." );
    }

    std::vector<mrg_slam_msgs::msg::KeyFrameRos> static_keyframes_ros;

    for( const auto &kf : static_keyframes_to_add ) {
        geometry_msgs::msg::Pose patch_center;
        patch_center.position.x    = kf->center.x;
        patch_center.position.y    = kf->center.y;
        patch_center.position.z    = kf->center.z;
        patch_center.orientation.x = 0.0;
        patch_center.orientation.y = 0.0;
        patch_center.orientation.z = 0.0;
        patch_center.orientation.w = 1.0;  // quaternion identity
        mrg_slam_msgs::msg::KeyFrameRos kf_ros;
        kf_ros.uuid_str        = kf->uuid_str;
        kf_ros.stamp           = now();
        kf_ros.robot_name      = "static_keyframe_provider";
        kf_ros.odom_counter    = -1;
        kf_ros.first_keyframe  = false;
        kf_ros.static_keyframe = true;
        kf_ros.slam_uuid_str   = kf->uuid_str;  // the slam uuid is the same as the static keyframe uuid
        kf_ros.accum_distance  = -1.0;
        kf_ros.estimate        = patch_center;
        kf_ros.cloud           = *kf->patch_msg;
        static_keyframes_ros.push_back( kf_ros );
    }

    mrg_slam_msgs::srv::AddStaticKeyFrames::Request::SharedPtr req = std::make_shared<mrg_slam_msgs::srv::AddStaticKeyFrames::Request>();
    req->keyframes                                                 = std::move( static_keyframes_ros );

    auto result_future = add_static_keyframes_clients[slam_pose_msg->robot_name]->async_send_request( req );

    std::future_status status = result_future.wait_for( std::chrono::seconds( 20 ) );
    if( status == std::future_status::timeout ) {
        RCLCPP_ERROR_STREAM( get_logger(), "Add static map service call to robot " << slam_pose_msg->robot_name << " timed out" );
    }
    if( status == std::future_status::ready ) {
        RCLCPP_INFO_STREAM( get_logger(), "Add static map service call to robot " << slam_pose_msg->robot_name << " successful" );
    }

    return result_future.get()->success;
}

void
StaticKeyFrameProviderComponent::publish_map_service( std_srvs::srv::Trigger::Request::ConstSharedPtr /* req */,
                                                      std_srvs::srv::Trigger::Response::SharedPtr res )
{
    RCLCPP_INFO_STREAM( get_logger(), "Publishing full map on " << map_pub->get_topic_name() );

    // Publish the full map
    map_pub->publish( *full_map );

    res->success = true;
}

void
StaticKeyFrameProviderComponent::publish_keyframes_service( std_srvs::srv::Trigger::Request::ConstSharedPtr /* req */,
                                                            std_srvs::srv::Trigger::Response::SharedPtr res )
{
    RCLCPP_INFO_STREAM( get_logger(), "Publishing map patches on topic " << keyframes_pub->get_topic_name() );
    RCLCPP_INFO_STREAM( get_logger(), "Publishing patch center points and origin on topic " << marker_pub->get_topic_name() );

    // Publish the center points
    publish_center_points();
    // Publish the patches by starting the keyframes_pub_timer
    keyframes_pub_timer->reset();

    res->success = true;
}

void
StaticKeyFrameProviderComponent::patch_pub_timer_callback()
{
    keyframes_pub_timer->cancel();

    auto                         patch = static_keyframes[patch_index]->patch;
    pcl::PointCloud<PointT>::Ptr patch_in_map( new pcl::PointCloud<PointT> );
    for( const auto &point : patch->points ) {
        PointT temp_point = point;
        temp_point.x += static_keyframes[patch_index]->center.x;
        temp_point.y += static_keyframes[patch_index]->center.y;
        temp_point.z += static_keyframes[patch_index]->center.z;
        patch_in_map->points.push_back( temp_point );
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg( *patch_in_map, msg );
    // Transform the patch back to the map frame
    msg.header.frame_id = frame_id;
    msg.header.stamp    = now();
    RCLCPP_INFO_STREAM( get_logger(), "Publishing patch " << patch_index + 1 << "/" << static_keyframes.size() << " with " << patch->size()
                                                          << " points in frame_id " << msg.header.frame_id );
    keyframes_pub->publish( msg );

    patch_index++;

    // Cancel the timer if all patches have been published once
    if( patch_index >= static_keyframes.size() ) {
        patch_index = 0;
        keyframes_pub_timer->cancel();
        return;
    }

    keyframes_pub_timer->reset();
}

void
StaticKeyFrameProviderComponent::publish_center_points()
{
    visualization_msgs::msg::MarkerArray markers;
    int                                  counter = 0;
    for( const auto &static_keyframe : static_keyframes ) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id    = frame_id;
        marker.header.stamp       = now();
        marker.ns                 = "patch_centers";
        marker.id                 = counter++;
        marker.type               = visualization_msgs::msg::Marker::SPHERE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = static_keyframe->center.x;
        marker.pose.position.y    = static_keyframe->center.y;
        marker.pose.position.z    = static_keyframe->center.z;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x            = 0.3;
        marker.scale.y            = 0.3;
        marker.scale.z            = 0.3;
        marker.color.a            = 1.0;  // Don't forget to set the alpha!
        marker.color.r            = 0.0;
        marker.color.g            = 1.0;
        marker.color.b            = 0.0;
        markers.markers.push_back( marker );
    }
    // add a fat Sphere marker at the origin
    visualization_msgs::msg::Marker origin_marker;
    origin_marker.header.frame_id    = frame_id;
    origin_marker.header.stamp       = now();
    origin_marker.ns                 = "patch_centers";
    origin_marker.id                 = counter++;
    origin_marker.type               = visualization_msgs::msg::Marker::SPHERE;
    origin_marker.action             = visualization_msgs::msg::Marker::ADD;
    origin_marker.pose.position.x    = 0;
    origin_marker.pose.position.y    = 0;
    origin_marker.pose.position.z    = 0;
    origin_marker.pose.orientation.x = 0.0;
    origin_marker.pose.orientation.y = 0.0;
    origin_marker.pose.orientation.z = 0.0;
    origin_marker.pose.orientation.w = 1.0;
    origin_marker.scale.x            = 0.5;
    origin_marker.scale.y            = 0.5;
    origin_marker.scale.z            = 0.5;  // make the origin marker fat so that it is visible even if the camera is zoomed out
    origin_marker.color.a            = 1.0;  // Don't forget to set the alpha!
    origin_marker.color.r            = 1.0;
    origin_marker.color.g            = 1.0;
    origin_marker.color.b            = 0.0;
    markers.markers.push_back( origin_marker );
    // add a circular marker for each patch with the patch radius
    for( const auto &static_keyframe : static_keyframes ) {
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header.frame_id    = frame_id;
        circle_marker.header.stamp       = now();
        circle_marker.ns                 = "patch_centers";
        circle_marker.id                 = counter++;
        circle_marker.type               = visualization_msgs::msg::Marker::LINE_STRIP;
        circle_marker.action             = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose.orientation.w = 1.0;
        circle_marker.scale.x            = 0.1;  // thickness
        circle_marker.color.r            = 1.0;
        circle_marker.color.a            = 0.8;

        double                 radius     = keyframe_radius;
        const Eigen::Vector3f &p_center   = static_keyframe->center.getVector3fMap();
        int                    num_points = 50;
        for( int i = 0; i <= num_points; i++ ) {
            double                    angle = 2.0 * M_PI * i / num_points;
            geometry_msgs::msg::Point p;
            p.x = p_center.x() + radius * std::cos( angle );
            p.y = p_center.y() + radius * std::sin( angle );
            p.z = p_center.z();
            circle_marker.points.push_back( p );
        }

        markers.markers.push_back( circle_marker );
    }

    marker_pub->publish( markers );
    RCLCPP_INFO_STREAM( get_logger(), "Published " << static_keyframes.size() << " center points and origin" );
}

}  // namespace mrg_slam

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE( mrg_slam::StaticKeyFrameProviderComponent )
