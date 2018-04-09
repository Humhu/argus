# camplex
Software for **cam**era multi**plexing**, fiducial pose estimation, and general camera/vision utilities.

# Dependencies
* [argus_utils](https://github.com/Humhu/argus_utils): Utilities for all argus packages
* OpenCV 3+: Used for pose estimation, calibration

# Class Overview
## CameraCalibration 
A convenience wrapper class for interfacing ROS-style camera_info messages and YAML files with OpenCV camera parameter formats. CameraCalibration extends the ROS pinhole_camera class with scaling and ROI, enabling calibrations generated at one resolution to be adapted to uniformly scaled images.

## CameraCalibrator
Wraps the OpenCV calibration routine to use the camplex fiducial messages. Breaks out the calibration parameters into human-readable struct fields and returns a CameraCalibration object.

## FiducialInfoManager
Provides an interface for reading/writing fiducial information to the ROS parameter server through the lookup::InfoManager abstraction. Refer to argus_utils/lookup for more information.

## FiducialVisualizer
Extends vizard::PoseVisualizer to provide methods for visualizing fiducial keypoints. See argus_utils/vizard for more information.

## DriverNode, CameraDriver
A libV4L/V4L2-based camera driver that exposes parameters with the paraset::ParameterManager abstraction. Mostly deprecated at this point, in favor of packages that support compressed video outputs.

## SplitStereoDriverNode
Wraps CameraDriver to split a side-by-side video stream from a stereo camera into two separate image topics. Useful in particular for the ZED camera.

# Node Overview
## camera_calibrator_node
WIP node for calibrating cameras using fiducials.

## camera_node
Camera driver node.

## checkerboard_detector_node
Outputs fiducial detections of a checkerboard from an image topic.

## checkerboard_registrar
Writes fiducial parameters to the ROS param server for a checkerboard fiducial.

## fiducial_pose_estimator
Outputs poses of individual or an array of fiducials using their intrinsics and extrinsic information.

## recorder_node
Records grayscale video as a sequence of imagess (TODO: Support rgb).

## resize_node
Allows dynamic image scaling. Optionally also outputs a scaled camera_info topic.

## split_camera_node
Camera driver node for split-stereo camera driver.

## undistortion_node
Caches an undistortion map upon initialization and publishes undistorted images.

## video_recorder
Records RGB video in a compressed format.

## viewer_node
Visualizes an image topic. Optionally filters images on the topic based on their header.frame_id.

# Message Overview
## FiducialInfo.msg
A message representation of ordered point-based fiducials.