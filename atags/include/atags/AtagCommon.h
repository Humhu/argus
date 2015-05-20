#ifndef _ATAGS_COMMON_H_
#define _ATAGS_COMMON_H_

#include "apriltags/TagDetection.h"

#include "argus_common/PoseSE3.h"
#include "argus_msgs/TagDetection.h"

namespace atags 
{
	/*! \brief Parses some fields into a native TagDetection object. */
	AprilTags::TagDetection msg_to_detection( const argus_msgs::TagDetection& msg );
	
	/*! \brief Return the pose of a rectified tag detection. */
	argus_common::PoseSE3 get_tag_pose( const argus_msgs::TagDetection& msg, double tagSize );
	argus_common::PoseSE3 get_tag_pose( const argus_msgs::TagDetection& msg, double tagSize,
										double fx, double fy, double px, double py );
	
}

#endif
