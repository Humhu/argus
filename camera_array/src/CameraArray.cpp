#include "camera_array/CameraArray.h"

#include "argus_utils/GeometryUtils.h"

#include <boost/foreach.hpp>

namespace camera_array
{

CameraArray::CameraArray() {}

void CameraArray::FromInfo( const CameraArrayInfo& info )
{
	if( info.cameraNames.size() != info.extrinsics.size() )
	{
		throw std::runtime_error( "Must have same number of array elements in CamerArrayInfo." );
	}
	
	
	for( unsigned int i = 0; i < info.cameraNames.size(); i++ )
	{
		CameraRegistration registration;
		registration.extrinsics = argus_utils::MsgToPose( info.extrinsics[i] );
// 		registration.cameraModel.fromCameraInfo( info.intrinsics[i] );
		cameraRegistry[ info.cameraNames[i] ] = registration;
	}
}

argus_utils::PoseSE3 CameraArray::GetCameraExtrinsics( const std::string& name ) const
{
	return cameraRegistry.at( name ).extrinsics;
}

// image_geometry::PinholeCameraModel CameraArray::GetCameraIntrinsics( const std::string& name ) const
// {
// 	return cameraRegistry.at( name ).cameraModel;
// }

CameraArray CameraArray::GetSubset( const std::vector< std::string >& names ) const
{
	CameraArray other;
	BOOST_FOREACH( const std::string& name, names )
	{
		// Should throw out_of_range exception if nonexistant
		other.cameraRegistry[ name ] = cameraRegistry.at( name );
	}
	return other;
}
	
} // end namespace camera_array
