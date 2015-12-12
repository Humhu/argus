#include "fieldtrack/TargetInfoManager.h"

namespace fieldtrack
{
	
TargetInfoManager::TargetInfoManager( lookup::LookupInterface& interface )
: InfoManager( interface )
{}

bool TargetInfoManager::ReadMemberInfo( const std::string& memberName,
                                        bool forceLookup )
{
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup ) ) { return false; }
	
	TargetInfo registration;
	
	// Parse motion mode
	std::string motionString;
	std::string motionKey = GenerateMotionModeKey( memberNamespace );
	if( !nodeHandle.getParam( motionKey, motionString ) )
	{
		ROS_WARN_STREAM( "Could not find motion mode information for: " << memberName
			<< " at path " << motionKey );
		RecordFailure( memberName );
		return false;
	}
	registration.motionMode = StringToMotion( motionString );
	if( registration.motionMode == INVALID )
	{
		ROS_WARN_STREAM( "Read invalid motion mode string of: " << motionString
			<< " for: " << memberName << " at path " << motionKey );
		return false;
	}
	
	// Parse fiducial names
	std::string fiducialsKey = GenerateFiducialsKey( memberNamespace );
	nodeHandle.param( fiducialsKey, registration.fiducialNames, std::vector<std::string>() );
	
	RegisterMember( memberName, registration );
	return true;
}

bool TargetInfoManager::WriteMemberInfo( const std::string& memberName,
                                         bool forceLookup )
{
	if( !HasMember( memberName ) ) { return false; }
	
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup ) ) { return false; }
	
	TargetInfo& registration = GetInfo( memberName );
	nodeHandle.setParam( GenerateMotionModeKey( memberNamespace ), 
	                     MotionToString( registration.motionMode ) );
	if( !registration.fiducialNames.empty() )
	{
		nodeHandle.setParam( GenerateFiducialsKey( memberNamespace ),
	                         registration.fiducialNames );
	}
	return true;
}

std::string TargetInfoManager::GenerateMotionModeKey( const std::string& ns )
{
	return ns + "motion_mode";
}

std::string TargetInfoManager::GenerateFiducialsKey( const std::string& ns )
{
	return ns + "fiducial_names";
}
	
TargetMotion StringToMotion( const std::string& m )
{
	if( m == "stationary" ) { return STATIONARY; }
	else if( m == "constant_velocity" ) { return CONSTANT_VELOCITY; }
	else if( m == "random" ) { return RANDOM; }
	else { return INVALID; }
}

std::string MotionToString( TargetMotion m )
{
	switch( m )
	{
		case STATIONARY: return "stationary";
		case CONSTANT_VELOCITY: return "constant_velocity";
		case RANDOM: return "random";
		default: return "invalid";
	}
}
	
}
