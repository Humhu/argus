# The AprilTag family this detection belongs to
string family

# The tag integer ID
int32 id

# The hamming distance between the observed and assigned ID in quads (bits)
int32 hammingDistance

# Whether the detected corners are undistorted
bool undistorted

# Whether the coordinates are in normalized image coordinates
bool normalized

# The image coordinates of the tag corners, counter-clockwise starting from lower left
Point2D_32[4] corners
