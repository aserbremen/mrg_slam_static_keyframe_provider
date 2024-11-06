# Static Keyframe Provider for mrg_slam instances

This package provides a static keyframe provider for mrg_slam instances. When you have a map, in particular a `.pcd` point cloud file, from a previously explored environment, you can use this package to provide keyframes to the mrg_slam instances. This node creates circular keyframes at a given grid step size and publishes them to the mrg_slam instances, whenever the robot is within a certain distance from the keyframe.

The static keyframe provider node is a central authority that creates keyframe with reproducable unique IDs from deterministic splitting of the point cloud into keyframes. This means that if a point cloud of a map is split into keyframes with the same grid step size, the keyframes will have the same unique IDs. This is needed so that each static keyframe is only published once to the mrg_slam instances.

Check out this video demonstrating the static keyframe provider in action:

<iframe src="https://drive.google.com/file/d/1OQlk74DVrMTw5Tp8H13Oq838VyekutZT/preview" width="640" height="480" allow="autoplay"></iframe>


