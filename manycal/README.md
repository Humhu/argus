# manycal
Package for **many**-camera and fiducial **cal**ibration.

# Dependencies
* [graphopt](https://github.com/Humhu/graphopt): Wrapper and extension of ISAM library
* [argus_utils](https://github.com/Humhu/argus_utils): Utilities for argus packages

# Concepts
## Simultaneous Calibration Localization and Mapping (SCLAM)
Manycal uses the fiducial-based SCLAM extension to ISAM found in graphopt. This means that you can, in theory, simultaneously optimize extrinsics and intrinsics of both cameras and fiducials. In practice, however, you will probably want to configure your optimization to only optimize certain variables.

## Arrays and Target Reference Frames
In manycal we refer to a group of cameras and fiducials as an *array* with a parent *target* reference frame. The extrinsics of array members are given with respect to their parent target. An example is a camera mounted on a robot. A natural setup is to have the target reference frame at the center of the robot, and the extrinsics of the camera be relative to this robot target frame. Another example is a fiducial mounted on a wall. The fiducial extrinsics would be relative to some fixed map reference frame.

## Target Motion Modes
Clearly, robots and walls don't exhibit the same dynamics, and manycal is able to incorporate this into the optimization with *motion modes*. Currently there are three supported motion modes:
* **static:** This denotes a target that is stationary for the lifetime of the optimization, and is well-suited for map frames or otherwise stationary arrays. The target will be represented with a single pose node.
* **dynamic:** This denotes a target that moves and publishes its body velocity on an odometry topic, and is well-suited for robots. The target poses will form a sequential chain of nodes joined by odometry factors.
* **discontinuous:** This denotes a target that moves erratically and does not publish its body velocity, and is well-suited for uninstrumented objects. The target poses will be a collection of wholly independent pose nodes, indexed by time.

## Toggling Optimization of Variables
Not all extrinsics and intrinsics have to be optimized. For instance, if we want to fix a particular fiducial in our map to be at the origin, we would not want the optimization to optimize that extrinsics variable. By default, all variables are **not** optimized unless specified. Further details are explained below in the parameter interface to `array_calibrator_node`.

## Initializing Variables
If prior information is available or if a variable is not to be optimized, it is helpful to provide an initialization. Currently only extrinsics initializations are supported for array members, as well as pose initializations for target frames. 

Note that a target pose initialization is applied at startup. This means that a dynamic target will have its first pose initialized at a very early time. If this time precedes any odometry messages for that target, later attempts to build the target odometry pose graph will fail, so it is currently not recommended to use pose initialization for dynamic targets.

## Camera Intrinsics Model
We use a rudimentary pinhole camera model with a separate focal lengths and a principal point. Thus, we recommend that you use normalized and undistorted fiducial detections.

All camera intrinsics are initialized to unity focal length and zero principal point. Currently we do not support initializing intrinsics with parameters.

## Fiducial Intrinsics Model
We represent fiducials as unique 3D points existing around some origin reference frame. The definition of the fiducial axes and intrinsics is detailed in camplex. Calibrating fiducials allows these points to move around in 3D space, and is best done with a prior to fix the fiducial frame's relative orientation.

All fiducial intrinsics are required to be readable from the global fiducial lookup system described in camplex (TODO!).

# Usage
## Specifying Poses
Manycal uses the argus_utils parameter tools to parse poses. In short, a pose is specified as nested parameters in either `x y z yaw pitch roll` Euler angle form or `x y z qw qx qy qz` quaternion form. For example:
```
pose_euler:
  x: 0.0
  y: 0.1
  z: -2.0
  yaw: 0.1
  pitch: -3.14
  roll: 0.0
  
 pose_quat:
   x: 0.0
   y: 0.1
   z: -2.0
   qw: 1.0
   qx: 0.0
   qy: 0.0
   qz: 0.0
```

## Specifying Covariances
Manycal uses the argus_utils parameter reading tools for loading covariance matrices. In short, diagonal N by N matrices can be specified with an array of N floats (inf and nan are parseable as well), and dense N by N matrices can be specified row-major with an array of N * N floats. For more details, refer to argus_utils.

## Specifying Optimization Configurations
Manycal uses private ROS parameters to read in an optimization specification. The particular root private namespace may vary from node to node, but is generally detailed below. Refer to the `launch` folder for an example.

### Common to All Targets
* `[target_name]`: The unique frame ID of the target reference frame for this array
* `[target_name]/type`: (string, 'static', 'dynamic', or 'discontinuous') The motion mode of this target
* `[target_name]/optimize_pose`: (bool, default False) Whether or not to optimize the target pose(s)
* `[target_name]/initial_pose`: (Pose) If given, initializes the target pose at startup time
* `[target_name]/prefix_members`: (bool, default False) If enabled, prefixes target_name to each of the array member names
* `[target_name]/cameras`: Nested camera specifications for all member cameras (see below)
* `[target_name]/fiducials`: Nested fiducial specifications for all member fiducials (see below)

### Dynamic Targets
* `[target_name]/odom_topic`: (string) The odometry topic for this target
* `[target_name]/integrator_buffer_len`: (float, default 10.0) How many seconds of odometry info to buffer
* `[target_name]/odom_buffer_len`: (unsigned int, default 10) The odometry topic subscriber buffer length

### Common to All Array Members
* `[member_name]/optimize_extrinsics`: (bool, default False) Whether or not to optimize the extrinsic pose
* `[member_name]/initial_extrinsics`: (Pose) If given, initializes the extrinsics and adds a prior
* `[member_name]/extrinsics_prior_cov`: (6x6 matrix, default 1e-3 * Identity) The extrinsics prior factor covariance

### Cameras
* `[member_name]/optimize_intrinsics`: (bool, default False) Whether or not to optimize the camera intrinsics
* `[member_name]/intrinsics_prior_cov`: (4x4 matrix, default 1e-3 * Identity) The intrinsics prior factor covariance

### Fiducials
* `[member_name]/optimize_intrinsics`: (bool, default False) Whether or not to optimize the fiducial intrinsics
* `[member_name]/intrinsics_prior_cov`: (3Nx3N matrix where N is the number of fiducial points, default 1e-3 * Identity) The intrinsics prior factor covariance

# Nodes
## array_calibrator_node
The general array calibration node. Reads in an optimization specification through the aforementioned format. Buffers and processes detections at a lag from realtime to allow for latency in topics. Also re-buffers detections that cannot be immediately used due to uninitialized variables. Uses PNP to estimate fiducial pose and initialize pose variables when appropriate (if only one pose in the pose chain is missing). Regularly prints out optimization status and saves all extrinsics to a YAML file on shutdown.

### Parameters
* `~graph`: Optional namespace to define graphopt/GraphOptimizer parameters
* `~extrinsics_init_cov`: (6x6 matrix, default is Identity) Covariance to use for prior when initializing member extrinsics from fiducial PNP estimates
* `~targets`: Namespace for optimization specification
* `~detection_topics`: (list of strings) Topics to subscribe to for argus_msgs/ImageFiducialDetections messages
* `~detection_buffer_len`: (unsigned int, default 10) Each detection subscriber's buffer length
* `~max_detection_buffer_lag`: (float, default 5.0) The max reprocessed detection age in seconds
* `~save_path`: (string) The output extrinsics YAML file path
* `~spin_lag`: (float) The processing lag in seconds
* `~spin_rate`: (float, default 1.0) The processing rate in Hz

# Tools
## udev-rules generator
Generates a udev rules file that assigns static symlinks to all video devices based on their USB hardware path. This means that cameras that are plugged into certain hubs and ports will always get linked to the same symlink. For more details refer to the script itself.
