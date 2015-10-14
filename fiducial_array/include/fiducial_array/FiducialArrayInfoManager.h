#pragma once

#include <ros/ros.h>

// Messages
#include "fiducial_array/FiducialArrayInfo.h"

#include <unordered_map>

namespace fiducial_array
{
	
/*! \brief Stores and retrieves/caches fiducial extrinsics information from the ROS
 * parameter server. */
class FiducialArrayInfoManager
{
public:
	
	FiducialArrayInfoManager( ros::NodeHandle& nh );
	
	/*! \brief Load information from the parameter server to the local cache. */
	// TODO Implement updates, partial reads
	void ReadFiducialInformation( const std::string& paramRoot );
	
	/*! \brief Lookup an array based on the array name. */
	const FiducialArrayInfo& LookupArray( const std::string& arrayName ) const;
	
	/*! \brief Lookup an array based on a member fiducial. */
	const FiducialArrayInfo& LookupFiducial( const std::string& fidName ) const;
	
private:
	
	ros::NodeHandle nodeHandle;

	/*! \brief Map from array names to info. */
	std::unordered_map< std::string, std::shared_ptr<FiducialArrayInfo> > arrayMap;
	
	/*! \brief Map from fiducial names to owning info. */
	std::unordered_map< std::string, std::shared_ptr<FiducialArrayInfo> > fidMap;
	
};
	
} // end namespace fiducial_array
