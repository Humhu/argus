# Represents a set of fiducial detections from a single image
# Timestamp and frame_id are from source image
std_msgs/Header header

# Detections found in the image
FiducialDetection[] detections
