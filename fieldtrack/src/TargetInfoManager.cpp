#include "fieldtrack/TargetInfoManager.h"
#include <boost/foreach.hpp>

#define MOTION_KEY "motion_mode"
#define ODOMETRY_KEY "odometry_topic"
#define DETECTION_KEY "detection_topic"
#define CAMERAGROUP_KEY "camera_groups"
#define CAMERAS_KEY "cameras"
#define FIDUCIALGROUP_KEY "fiducial_groups"
#define FIDUCIALS_KEY "fiducials"

namespace fieldtrack
{
	
TargetInfoManager::TargetInfoManager( lookup::LookupInterface& interface )
: InfoManager( interface )
{}

bool TargetInfoManager::ReadMemberInfo( const std::string& memberName,
                                        bool forceLookup,
                                        const ros::Duration& timeout )
{
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}
	
	TargetInfo registration;
	
	// Parse motion mode
	std::string motionString;
	std::string motionKey = memberNamespace + MOTION_KEY;
	if( !nodeHandle.getParam( motionKey, motionString ) )
	{
		ROS_WARN_STREAM( "Could not find motion mode information for: " << memberName
			<< " at path " << motionKey );
		RecordFailure( memberName );
		return false;
	}
	registration.motionMode = StringToMotion( motionString );
	if( registration.motionMode == MOTION_INVALID )
	{
		ROS_WARN_STREAM( "Read invalid motion mode string of: " << motionString
			<< " for: " << memberName << " at path " << motionKey );
		return false;
	}
	
	// Parse odometry topic
	std::string odomTopic;
	if( nodeHandle.getParam( memberNamespace + ODOMETRY_KEY, odomTopic ) )
	{
		if( odomTopic.front() == '/' ) { registration.odometryTopic = odomTopic; }
		else { registration.odometryTopic = memberNamespace + odomTopic; }
	}

	// Parse odometry topic
	std::string detTopic;
	if( nodeHandle.getParam( memberNamespace + DETECTION_KEY, detTopic ) )
	{
		if( detTopic.front() == '/' ) { registration.detectionTopic = detTopic; }
		else { registration.detectionTopic = memberNamespace + detTopic; }
	}

	// Parse camera groups
	std::vector<std::string> cameraGroups;
	if( nodeHandle.getParam( memberNamespace + CAMERAGROUP_KEY, cameraGroups ) )
	{
		BOOST_FOREACH( const std::string& group, cameraGroups )
		{
			CameraGroupInfo info;
			if( !ParseCameraGroup( memberNamespace + group + "/", info ) )
			{
				ROS_WARN_STREAM( "Could not parse camera group info for: " << memberName
				                 << " group: " << group );
				return false;
			}
			registration.cameraGroups[ group ] = info;
		}
	}

	// Parse fiducial groups
	std::vector<std::string> fiducialGroups;
	if( nodeHandle.getParam( memberNamespace + FIDUCIALGROUP_KEY, fiducialGroups ) )
	{
		BOOST_FOREACH( const std::string& group, fiducialGroups )
		{
			FiducialGroupInfo info;
			if( !ParseFiducialGroup( memberNamespace + group + "/", info ) )
			{
				ROS_WARN_STREAM( "Could not parse fiducial group info for: " << memberName
				                 << " group: " << group );
				return false;
			}
			registration.fiducialGroups[ group ] = info;
		}
	}
	
	RegisterMember( memberName, registration );
	return true;
}

bool TargetInfoManager::WriteMemberInfo( const std::string& memberName,
                                         bool forceLookup,
                                         const ros::Duration& timeout )
{
	if( !HasMember( memberName ) ) { return false; }
	
	std::string memberNamespace;
	if( !GetNamespace( memberName, memberNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}
	
	TargetInfo& registration = GetInfo( memberName );
	nodeHandle.setParam( memberNamespace + MOTION_KEY,
	                     MotionToString( registration.motionMode ) );
	
	// TODO Write other parameters

	return true;
}

bool TargetInfoManager::ParseCameraGroup( const std::string& ns, 
                                          CameraGroupInfo& info )
{
	return nodeHandle.getParam( ns + CAMERAS_KEY, info.cameras );
}

bool TargetInfoManager::ParseFiducialGroup( const std::string& ns, 
                                            FiducialGroupInfo& info )
{
	return nodeHandle.getParam( ns + FIDUCIALS_KEY, info.fiducials );
}

std::string MotionToString( MotionMode mode )
{
	switch( mode )
	{
		case MOTION_INDEPENDENT:
			return "independent";
		case MOTION_STATIC:
			return "static";
		case MOTION_MOVING:
			return "moving";
		default:
			return "invalid";
	}
}

MotionMode StringToMotion( const std::string& mode )
{
	if( mode == "independent" ) { return MOTION_INDEPENDENT; }
	if( mode == "static" ) { return MOTION_STATIC; }
	if( mode == "moving" ) { return MOTION_MOVING; }
	return MOTION_INVALID;
}
	
}
