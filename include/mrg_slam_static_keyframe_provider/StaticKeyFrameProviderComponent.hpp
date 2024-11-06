// SPDX-License-Identifier: BSD-2-Clause

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
// boost
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
// pcl
#include <pcl/point_cloud.h>
// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
// mrg_slam_msgs
#include <mrg_slam_msgs/msg/key_frame_ros.hpp>
#include <mrg_slam_msgs/msg/pose_with_name.hpp>
#include <mrg_slam_msgs/srv/add_static_key_frames.hpp>
#include <mrg_slam_msgs/srv/get_graph_gids.hpp>

namespace mrg_slam {

class StaticKeyFrameProviderComponent : public rclcpp::Node {
public:
    typedef pcl::PointXYZI PointT;
    struct StaticKeyframe {
        using Ptr = std::shared_ptr<StaticKeyframe>;
        PointT                                   center;
        pcl::PointCloud<PointT>::Ptr             patch;
        sensor_msgs::msg::PointCloud2::SharedPtr patch_msg;
        mrg_slam_msgs::msg::KeyFrameRos          key_frame;
        boost::uuids::uuid                       uuid;
        std::string                              uuid_str;
    };

    StaticKeyFrameProviderComponent( const rclcpp::NodeOptions &options );

    /**
     * @brief Creates a vector of circular static keyframes with a given grid step size and patch radius
     */
    void create_static_keyframes();

    /**
     * @brief Generates a reproducible UUID from deterministic keyframe information
     *
     * @param x x-coordinate of the keyframe center
     * @param y y-coordinate of the keyframe center
     * @param num_points number of points in the patch
     * @return std::string UUID as string
     */
    std::string uuid_str_from_keyframe( double x, double y, size_t num_points );

    /**
     * @brief Get the graph gids service call object
     *
     * @param robot_name
     * @return mrg_slam_msgs::srv::GetGraphGids::Response::SharedPtr Containing the already existing keyframes
     */
    mrg_slam_msgs::srv::GetGraphGids::Response::SharedPtr get_graph_gids_service_call( const std::string &robot_name );

    /**
     * @brief Adds the static
     *
     * @param robot_name
     * @return true if the service call was successful
     */
    bool add_static_keyframes_service_call( mrg_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg,
                                            const std::vector<StaticKeyframe::Ptr>          &static_keyframes_to_add );

    /**
     * @brief Broadcast callback for the slam poses of the multi-robot graph SLAM. Static map patches will be supplied dependent on the
     * distance to the robot.
     *
     * @param slam_pose_msg slam pose message containing the robot name and the pose
     */
    void slam_pose_broadcast_callback( mrg_slam_msgs::msg::PoseWithName::ConstSharedPtr slam_pose_msg );

    /**
     * @brief Trigger service callback for publishing the full map
     *
     * @param req
     * @param res
     */
    void publish_map_service( std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res );

    /**
     * @brief Trigger service callback for publishing the static keyframes timer based
     *
     * @param req
     * @param res
     */
    void publish_keyframes_service( std_srvs::srv::Trigger::Request::ConstSharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res );

    /**
     * @brief Timer callback for publishing the patches
     */
    void patch_pub_timer_callback();

    /**
     * @brief  Publishes the center points of the patches and origin of the static map
     */
    void publish_center_points();


private:
    std::vector<StaticKeyframe::Ptr> static_keyframes;

    // ROS2 params
    std::string              map_frame;
    float                    grid_step_size;
    float                    patch_radius;
    std::string              pcd_path;
    std::vector<std::string> robot_names;
    float                    slam_distance, squared_slam_distance;

    // unique id generators
    boost::uuids::random_generator_pure uuid_generator;

    sensor_msgs::msg::PointCloud2::SharedPtr full_map;

    // ROS2 subscribers
    rclcpp::Subscription<mrg_slam_msgs::msg::PoseWithName>::SharedPtr slam_pose_broadcast_sub;

    // ROS2 publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        keyframes_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;

    // ROS2 service clients
    std::unordered_map<std::string, rclcpp::Client<mrg_slam_msgs::srv::GetGraphGids>::SharedPtr>       get_graph_gids_clients;
    std::unordered_map<std::string, rclcpp::Client<mrg_slam_msgs::srv::AddStaticKeyFrames>::SharedPtr> add_static_keyframes_clients;

    // ROS2 service servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_map_service_server;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_keyframes_service_server;

    // ROS2 callback groups
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group1;
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group2;

    // More parameters
    size_t                       patch_index;
    rclcpp::TimerBase::SharedPtr keyframes_pub_timer;
    double                       timer_frequency;
};

}  // namespace mrg_slam