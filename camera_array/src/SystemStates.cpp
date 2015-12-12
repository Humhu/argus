#include "camera_array/SystemStates.h"

namespace camera_array
{

bool CameraArrayState::IsActive( const std::string& name ) const
{
	return activeCameras.count( name ) > 0;
}

bool CameraArrayState::IsInactive( const std::string& name ) const
{
	return inactiveCameras.count( name ) > 0;
}
	
bool CameraArrayState::Deactivate( const std::string& name )
{
	CameraSet::iterator iter = activeCameras.find( name );
	if( iter == activeCameras.end() ) { return false; }
	activeCameras.erase( iter );
	inactiveCameras.insert( name );
	return true;
}

bool CameraArrayState::Activate( const std::string& name )
{
	CameraSet::iterator iter = inactiveCameras.find( name );
	if( iter == inactiveCameras.end() ) { return false; }
	inactiveCameras.erase( iter );
	activeCameras.insert( name );
	return true;
}

AccelerationAction::AccelerationAction()
: acceleration( fieldtrack::TargetState::VelocityType::Zero() )
{}

}
