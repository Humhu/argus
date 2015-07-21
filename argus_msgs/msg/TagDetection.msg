# Header is derived from image source
std_msgs/Header header

# Image source properties
uint32 sourceWidth
uint32 sourceHeight

# Detection properties
string family
int32 id
int32 hammingDistance
bool isNormalized
Point2D_32[4] corners
Point2D_32 center
