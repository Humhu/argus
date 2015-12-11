#pragma once

#include <ros/ros.h>

#include "lookup/InfoManager.h"
#include "fiducials/Fiducial.h"

namespace fiducials
{
	
/*! \brief Stores and retrieves/caches fiducial intrinsics information from the ROS
 * parameter server. Uses the lookup interface. */
class FiducialInfoManager
: public lookup::InfoManager<Fiducial>
{
public:
	
	FiducialInfoManager( lookup::LookupInterface& interface );
	
	virtual bool ReadMemberInfo( const std::string& fidName, bool forceLookup = false );
	virtual bool WriteMemberInfo( const std::string& fidName, bool forceLookup = false );
		
private:
	
	ros::NodeHandle nodeHandle;
	
	static std::string GenerateIntrinsicsKey( const std::string& ns );
	
};
	
} // end namespace fiducials
