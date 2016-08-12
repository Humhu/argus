# Describes the action taken by a discrete policy

# stamp - The time the action was executed
# frame_id - The policy name
std_msgs/Header header

# The input at the time
float64[] input

# The action index executed
uint32 index