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
	 * server unles forceLookup is set. Returns success. */
	bool ReadFiducialInformation( const std::string& fidName, bool forceLookup = false );
	
	/*! \brief Write the cached intrinsics for a member fiducial. Returns success. */
	bool WriteFiducialInformation( const std::string& fidName, bool forceLookup = false );
	
	/*! \brief Returns whether the fiducial is cached. */
	bool HasFiducial( const std::string& fidName ) const;
	
	/*! \brief Retrieve the intrinsics for a member fiducial from the cache. 
	 * Throws an exception if the fiducial is not cached. */
	const Fiducial& GetIntrinsics( const std::string& fidName );
	
	/*! \brief Sets the cached intrinsics for a member fiducial. */
	void SetIntrinsics( const std::string& fidName, const Fiducial& fid );
	
private:
	
	ros::NodeHandle nodeHandle;
	lookup::LookupInterface& lookupInterface;
	
	std::unordered_set <std::string> failedQueries;
	std::unordered_map <std::string, Fiducial> fiducials;
	
	bool GetNamespace( const std::string& fidName, std::string& ns, bool forceLookup );
	static std::string GenerateIntrinsicsKey( const std::string& ns );
	
};
	
} // end namespace fiducials
