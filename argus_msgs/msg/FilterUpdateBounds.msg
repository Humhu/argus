# Message containing bounds info for a filter update step

# stamp - Time at which the observation was taken
# frame_id - This observation source's name
std_msgs/Header header

# +- 3 std dev bounds centered on the observation values
float64[] bounds_centered_upper
float64[] bounds_centered_lower

# +- 3 std dev bounds centered on 0
float64[] bounds_zeroed_upper
float64[] bounds_zeroed_lower