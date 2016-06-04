# Message containing a timestamped and named feature vector

# stamp - Time at which this feature was generated
# frame_id - This feature source's unique name
std_msgs/Header header

# The feature vector
float64[] features