# Message detailing a filter predict or update step

# stamp - Time at which this step ends at
# frame_id - This observation source's name [unused for predict]
std_msgs/Header header

uint64 step_num

# Whether this is a predict or update step
bool isPredict

# Estimate covariance before step
MatrixFloat64 Spre

# Q matrix for predict steps
# R matrix for update steps
MatrixFloat64 noiseCov

# Predict step fields
# Transition matrix that evolves covariance
MatrixFloat64 F

# The length of this predict step
float64 dt

# Update step fields
# Observation matrix that maps state covariance into measurement space
MatrixFloat64 H

# The observation prediction error
float64[] innovation

float64[] xpre
float64[] delta_x
float64[] observation
MatrixFloat64 Spost

# The innovation after the update step
float64[] post_innovation
