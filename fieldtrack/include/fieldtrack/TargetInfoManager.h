#pragma once

#include "lookup/InfoManager.h"
#include <unordered_map>
#include <unordered_set>

namespace fieldtrack
{

enum TargetMotion
{
	INVALID = 0,
	STATIONARY,
	CONSTANT_VELOCITY,
	RANDOM
};
	
TargetMotion StringToMotion( const std::string& m );
std::string MotionToString( TargetMotion m );

struct TargetInfo
{
	TargetMotion motionMode;
	std::vector <std::string> fiducialNames;
};

class TargetInfoManager
: public lookup::InfoManager<TargetInfo>
{
public:

	TargetInfoManager( lookup::LookupInterface& interface );
	
	virtual bool ReadMemberInfo( const std::string& memberName, bool forceLookup = false );
	
	virtual bool WriteMemberInfo( const std::string& memberName, bool forceLookup = false );
	
protected:
	
	ros::NodeHandle nodeHandle;
	
	static std::string GenerateMotionModeKey( const std::string& ns );
	static std::string GenerateFiducialsKey( const std::string& ns );

	
};
	
}
