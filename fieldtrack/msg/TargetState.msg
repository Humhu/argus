# TargetState
# ===============
# Description
# -----------
# Describes the state of a state estimation target.
#
# Covariance matrix de/serialization can be row or column major 
# since the matrix is symmetric.
#
# Fields
# ------
# header/stamp - Time of observation
# header/frame_id - Name of reference frame
# body_frame_id - Name of target body frame
# pose - The mean pose estimate
# derivatives - All tracked body-frame pose derivatives
# covariance - The full covariance matrix

std_msgs/Header header
string body_frame_id
geometry_msgs/Pose pose
float64[] derivatives
float64[] covariance