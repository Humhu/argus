# Describes the action taken by a continuous policy

# stamp - The time the action was executed
# frame_id - The policy name
std_msgs/Header header

# The inputs at the time
float64[] inputs

# The outputs executed
float64[] outputs