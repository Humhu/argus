#include "camera_array/SystemStates.h"

namespace argus
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
: acceleration( TargetState::VelocityType::Zero() )
{}

std::ostream& operator<<( std::ostream& os, const CameraArrayAction& action )
{
  switch( action.type )
  {
  case CameraArrayAction::DO_NOTHING:
    os << "(Do nothing)";
  break;
 case CameraArrayAction::ACTIVATE_CAMERA:
   os << "(Activate " << action.toActivate << ")";
   break;
 case CameraArrayAction::DEACTIVATE_CAMERA:
   os << "(Deactivate " << action.toDeactivate << ")";
   break;
 case CameraArrayAction::SWITCH_CAMERAS:
   os << "(Switch " << action.toDeactivate << " to " << action.toActivate << ")";
   break;
}
  return os;
}

}
