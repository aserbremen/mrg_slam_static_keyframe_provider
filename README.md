# Static Keyframe Provider for mrg_slam

This package provides a static keyframe provider for mrg_slam instances. When you have a map, in particular a `.pcd` point cloud file, from a previously explored environment, you can use this package to provide keyframes to the mrg_slam instances. This node creates circular keyframes at a given grid step size and publishes them to the mrg_slam instances (see [Multi-Robot-Graph-SLAM](https://github.com/aserbremen/Multi-Robot-Graph-SLAM)), whenever the robot is within a certain distance from the keyframe.

The static keyframe provider node is a central authority that creates keyframe with reproducable unique IDs from deterministic splitting of the point cloud into keyframes. This means that if a point cloud of a map is split into keyframes with the same grid step size, the keyframes will have the same unique IDs. This is needed so that each static keyframe is only published once to the mrg_slam instances.

Check out a visualization of how a point cloud is split into circular static keyframes: 

https://github.com/user-attachments/assets/35a59782-8868-4d7d-bc88-cb941481bfa8

Check out this video demonstrating the static keyframe provider supplying keyframes to a robot running mrg_slam:

https://github.com/user-attachments/assets/d110955f-f775-4f9b-99f6-f7415482f745

## Usage

Setup the parameters of static keyframe provider node in the `config/static_keyframe_provider.yaml` file. The most important parameters are:

- `grid_step_size`: The distance between the centers of two adjacent keyframes placed on a x-y-grid.
- `keyframe_radius`: The radius of the circular keyframe.
- `frame_id`: The frame id of the point cloud.
- `pcd_path`: The path to the point cloud file in `.pcd` format. 
- `robot_names`: The names of the robots/namespaces that are running mrg_slam instances.
- `slam_distance`: The distance from the keyframe at which the keyframe is published to the mrg_slam instances.

To start the static keyframe provider node, run:

```bash
ros2 launch mrg_slam_static_keyframe_provider static_keyframe_provider.launch.py
```

## Visualization

You can visualize the full map and the keyframes in RViz by running:

```bash
# Publish full map on /static_keyframe_provider/map topic
ros2 service call /static_keyframe_provider/publish_map std_srvs/srv/Trigger 
# Publish keyframes on /static_keyframe_provider/keyframes topic periodically in a timer
ros2 service call /static_keyframe_provider/publish_keyframes std_srvs/srv/Trigger
```
