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
	
	virtual bool ReadMemberInfo( const std::string& fidName, 
	                             bool forceLookup = false,
	                             const ros::Duration& timeout = ros::Duration( 0 ) );
	virtual bool WriteMemberInfo( const std::string& fidName, 
	                              bool forceLookup = false,
	                              const ros::Duration& timeout = ros::Duration( 0 ) );
	
private:
	
	ros::NodeHandle nodeHandle;
	
	static std::string GenerateIntrinsicsKey( const std::string& ns );
	
};
	
} // end namespace argus
