# Contains aggregated information for a camera array

# Names of the member cameras
string[] cameraNames

# The intrinsics of the member cameras
sensor_msgs/CameraInfo[] intrinsics

# The relative poses of the member cameras in the array reference frame
geometry_msgs/Pose[] extrinsics
