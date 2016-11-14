# FilterState
# ===============
# Description
# -----------
# Describes the state of a state estimation filter.
#
# Covariance matrix de/serialization can be row or column major 
# since the matrix is symmetric.
#
# Fields
# ------
# header/stamp - Time of observation
# header/frame_id - Name of observation source
# pose - The mean pose estimate
# derivatives - All tracked body-frame pose derivatives
# covariance - The full covariance matrix

std_msgs/Header header
geometry_msgs/Pose pose
float64[] derivatives
float64[] covariance