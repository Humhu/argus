#include "fieldtrack/TargetInfoManager.h"
#include <boost/foreach.hpp>

#define MOTION_KEY "motion_mode"
#define ODOMETRY_KEY "odometry_topic"
#define DETECTION_KEY "detection_topic"
#define CAMERAGROUP_KEY "camera_groups"
#define CAMERAS_KEY "cameras"
#define FIDUCIALGROUP_KEY "fiducial_groups"
#define FIDUCIALS_KEY "fiducials"

namespace argus
{
	
TargetInfoManager::TargetInfoManager( LookupInterface& interface )
: InfoManager( interface )
{}

bool TargetInfoManager::ParseMemberInfo( const std::string& memberNamespace,
                                         TargetInfo& info )
{
	// Parse motion mode
	std::string motionString;
	std::string motionKey = memberNamespace + MOTION_KEY;
	if( !_nodeHandle.getParam( motionKey, motionString ) )
	{
		ROS_WARN_STREAM( "Could not find motion mode information at path " << motionKey );
		return false;
	}
	info.motionMode = StringToMotion( motionString );
	if( info.motionMode == MOTION_INVALID )
	{
		ROS_WARN_STREAM( "Read invalid motion mode string of: " << motionString
			<< " at path " << motionKey );
		return false;
	}
	
	// Parse odometry topic
	std::string odomTopic;
	if( _nodeHandle.getParam( memberNamespace + ODOMETRY_KEY, odomTopic ) )
	{
		if( odomTopic.front() == '/' ) { info.odometryTopic = odomTopic; }
		else { info.odometryTopic = memberNamespace + odomTopic; }
	}

	// Parse odometry topic
	std::string detTopic;
	if( _nodeHandle.getParam( memberNamespace + DETECTION_KEY, detTopic ) )
	{
		if( detTopic.front() == '/' ) { info.detectionTopic = detTopic; }
		else { info.detectionTopic = memberNamespace + detTopic; }
	}

	// Parse camera groups
	std::vector<std::string> cameraGroups;
	if( _nodeHandle.getParam( memberNamespace + CAMERAGROUP_KEY, cameraGroups ) )
	{
		BOOST_FOREACH( const std::string& group, cameraGroups )
		{
			CameraGroupInfo cginfo;
			if( !ParseCameraGroup( memberNamespace + group + "/", cginfo ) )
			{
				ROS_WARN_STREAM( "Could not parse camera group info for group: " << group );
				return false;
			}
			info.cameraGroups[ group ] = cginfo;
		}
	}

	// Parse fiducial groups
	std::vector<std::string> fiducialGroups;
	if( _nodeHandle.getParam( memberNamespace + FIDUCIALGROUP_KEY, fiducialGroups ) )
	{
		BOOST_FOREACH( const std::string& group, fiducialGroups )
		{
			FiducialGroupInfo fginfo;
			if( !ParseFiducialGroup( memberNamespace + group + "/", fginfo ) )
			{
				ROS_WARN_STREAM( "Could not parse fiducial group info for group: " << group );
				return false;
			}
			info.fiducialGroups[ group ] = fginfo;
		}
	}
	return true;
}

void TargetInfoManager::PopulateMemberInfo( const TargetInfo& info,
                                            const std::string& memberNamespace )
{
	_nodeHandle.setParam( memberNamespace + MOTION_KEY,
	                     MotionToString( info.motionMode ) );
	
	// TODO Write other parameters
}

bool TargetInfoManager::ParseCameraGroup( const std::string& ns, 
                                          CameraGroupInfo& info )
{
	return _nodeHandle.getParam( ns + CAMERAS_KEY, info.cameras );
}

bool TargetInfoManager::ParseFiducialGroup( const std::string& ns, 
                                            FiducialGroupInfo& info )
{
	return _nodeHandle.getParam( ns + FIDUCIALS_KEY, info.fiducials );
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
