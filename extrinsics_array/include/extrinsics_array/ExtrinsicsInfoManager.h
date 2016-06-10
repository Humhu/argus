#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "lookup/InfoManager.h"

#include <ros/ros.h>

namespace argus
{
	
struct ExtrinsicsInfo 
{
	std::string referenceFrame;
	PoseSE3 extrinsics;
};
	
/*! \brief Stores, retrieves, and caches extrinsics information from the ROS
 * parameter server. */
// TODO Provide services for saving extrinsics to disk?
class ExtrinsicsInfoManager
: public InfoManager<ExtrinsicsInfo>
{
public:
	
	ExtrinsicsInfoManager( LookupInterface& interface );

protected:
	
	ros::NodeHandle _nodeHandle;
	
	virtual bool ParseMemberInfo( const std::string& memberNamespace, 
	                              ExtrinsicsInfo& info );
	virtual void PopulateMemberInfo( const ExtrinsicsInfo& info,
	                                 const std::string& memberNamespace );
	
	static std::string GenerateExtrinsicsKey( const std::string& ns );
	static std::string GenerateRefFrameKey( const std::string& ns );
	
};
	
} // end namespace argus
