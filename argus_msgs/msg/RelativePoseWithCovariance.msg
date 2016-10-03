# TODO Rename to have Compact in the name?
# TODO Rename to have Stamped in the name?
# TODO Perhaps just rename to RelativePoseObservation?
# Message header
std_msgs/Header header

# The relative pose
argus_msgs/RelativePose relative_pose

# Covariance associated with relative pose
float64[36] covariance
