#include "camera_array/ArrayUtils.h"
#include "camplex/GetCameraInfo.h"

#include <boost/foreach.hpp>
#include <ros/ros.h>

using namespace camplex;

namespace camera_array
{

bool QueryInfo( const std::vector< std::string >& cameraNames,
				std::vector< sensor_msgs::CameraInfo >& info )
{
	info.clear();
	info.reserve( cameraNames.size() );
	BOOST_FOREACH( const std::string& name, cameraNames )
	{
		GetCameraInfo gci;
		if( !ros::service::call( name + "/get_camera_info", gci ) )
		{
			return false;
		}
		
		info.push_back( gci.response.info );
	}
	return true;
}
	
} // end namespace camera_array
