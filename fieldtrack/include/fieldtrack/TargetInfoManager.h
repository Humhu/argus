#pragma once

#include "lookup/InfoManager.h"
#include <unordered_map>
#include <unordered_set>

namespace argus
{

enum MotionMode
{
	MOTION_INVALID = 0,
	MOTION_INDEPENDENT,
	MOTION_STATIC,
	MOTION_MOVING
};
	
std::string MotionToString( MotionMode mode );
MotionMode StringToMotion( const std::string& mode );

struct CameraGroupInfo
{
	std::vector<std::string> cameras;
};

struct FiducialGroupInfo
{
	std::vector<std::string> fiducials;
};

struct TargetInfo
{
	MotionMode motionMode;

	std::string odometryTopic;
	std::string detectionTopic;

	typedef std::unordered_map <std::string, CameraGroupInfo> CameraGroupRegistry;
	CameraGroupRegistry cameraGroups;
	typedef std::unordered_map <std::string, FiducialGroupInfo> FiducialGroupRegistry;
	FiducialGroupRegistry fiducialGroups;
};

class TargetInfoManager
: public InfoManager<TargetInfo>
{
public:

	TargetInfoManager( LookupInterface& interface );
	

protected:
	
	ros::NodeHandle _nodeHandle;

	virtual bool ParseMemberInfo( const std::string& memberNamespace, 
	                              TargetInfo& info );
	
	virtual void PopulateMemberInfo( const TargetInfo& info,
	                                 const std::string& memberNamespace );
	
	bool ParseCameraGroup( const std::string& ns, CameraGroupInfo& info );
	bool ParseFiducialGroup( const std::string& ns, FiducialGroupInfo& info );
	
};
	
}
