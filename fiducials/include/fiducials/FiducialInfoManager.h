#pragma once

#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>

#include "lookup/LookupInterface.h"

#include "fiducials/Fiducial.h"

namespace fiducials
{

/*! \brief Stores and retrieves/caches fiducial intrinsics information from the ROS
 * parameter server. Uses the lookup interface. */
class FiducialInfoManager
{
public:
	
	FiducialInfoManager( lookup::LookupInterface& interface );
	
	/*! \brief Load information for a fiducial array from the parameter server 
	 * to the local cache. Overwrites existing cached values. If arrayPath has 
	 * been queried before and failed, this method skips querying the parameter 
	 * server unles forceRead is set. Returns success. */
	bool ReadFiducialInformation( const std::string& fidName, bool forceRead = false );
	
	/*! \brief Returns whether the fiducial is cached. */
	bool HasFiducial( const std::string& fidName ) const;
	
	/*! \brief Retrieve the intrinsics for a member fiducial. Throws an exception
	 * if the fiducial is not cached. */
	const Fiducial& GetIntrinsics( const std::string& fidName );
	
private:
	
	ros::NodeHandle nodeHandle;
	lookup::LookupInterface& lookupInterface;
	
	std::unordered_set <std::string> failedQueries;
	std::unordered_map <std::string, Fiducial> fiducials;
	
};
	
} // end namespace fiducials
