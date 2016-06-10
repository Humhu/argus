#pragma once

#include <ros/ros.h>

#include "lookup/InfoManager.h"
#include "fiducials/Fiducial.h"

namespace argus
{
	
/*! \brief Stores and retrieves/caches fiducial intrinsics information from the ROS
 * parameter server. Uses the lookup interface. */
class FiducialInfoManager
: public InfoManager<Fiducial>
{
public:
	
	FiducialInfoManager( LookupInterface& interface );
	
private:
	
	ros::NodeHandle _nodeHandle;
	
	virtual bool ParseMemberInfo( const std::string& memberNamespace, 
	                              Fiducial& info );
	virtual void PopulateMemberInfo( const Fiducial& info,
	                                 const std::string& memberNamespace );
	
	static std::string GenerateIntrinsicsKey( const std::string& ns );
	
};
	
} // end namespace argus
