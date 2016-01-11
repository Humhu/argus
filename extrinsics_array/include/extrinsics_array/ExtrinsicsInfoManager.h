#pragma once

#include "argus_utils/PoseSE3.h"
#include "lookup/InfoManager.h"

#include <ros/ros.h>

namespace extrinsics_array
{
	
struct ExtrinsicsInfo 
{
	std::string referenceFrame;
	argus_utils::PoseSE3 extrinsics;
};
	
/*! \brief Stores, retrieves, and caches extrinsics information from the ROS
 * parameter server. */
// TODO Provide services for saving extrinsics to disk?
class ExtrinsicsInfoManager
: public lookup::InfoManager<ExtrinsicsInfo>
{
public:
	
	ExtrinsicsInfoManager( lookup::LookupInterface& interface );

	/*! \brief Load information for the array that contains the specified member using
	 * the lookup table. Overwrites existing cached values. If memberName has been
	 * queried before and failed, this method skips querying the parameter server unless
	 * forceLookup is set. Returns success. */
	virtual bool ReadMemberInfo( const std::string& memberName, 
	                             bool forceLookup = false,
	                             const ros::Duration& timeout = ros::Duration( 0 ) );
	virtual bool WriteMemberInfo( const std::string& memberName, 
	                             bool forceLookup = false,
	                             const ros::Duration& timeout = ros::Duration( 0 ) );
	
protected:
	
	ros::NodeHandle nodeHandle;
	
	static std::string GenerateExtrinsicsKey( const std::string& ns );
	static std::string GenerateRefFrameKey( const std::string& ns );
	
};
	
} // end namespace extrinsics_array
