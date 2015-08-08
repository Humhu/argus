#pragma once

#include <sensor_msgs/CameraInfo.h>

namespace camera_array
{

/*! \brief Query each of the cameras for their sensor info. */
bool QueryInfo( const std::vector< std::string >& cameraNames,
				std::vector< sensor_msgs::CameraInfo >& info );

} // end namespace camera_array
