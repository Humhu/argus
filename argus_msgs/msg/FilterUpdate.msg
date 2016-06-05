# Message containing info for a filter update step

# stamp - Time at which the observation was taken
# frame_id - This observation source's name
std_msgs/Header header

# Observation matrix
MatrixFloat64 observation_matrix

# Observation covariance
# TODO Make SymmetricFloat64
MatrixFloat64 observation_cov

# The observation in vector form
float64[] observation