# Contains aggregated information for a fiducial array

# Names of the member fiducials
string[] fiducialNames

# The intrinsics of the member fiducials
FiducialInfo[] intrinsics

# The relative poses of the member fiducials in the array reference frame
geometry_msgs/Pose[] extrinsics
