#pragma once

#include <ros/ros.h>
#include <unordered_map>

#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "fiducial_array/FiducialArray.h"

namespace fiducial_array
{

/*! \brief Stores and retrieves/caches fiducial array information from the ROS
 * parameter server. Piggybacks on the extrinsics parameter lookup interface. */
class FiducialInfoManager
: public extrinsics_array::ExtrinsicsInfoManager
{
public:
	
	FiducialInfoManager( ros::NodeHandle& nh );
	
	virtual ~FiducialInfoManager();
	
	/*! \brief Load information for a fiducial array from the parameter server 
	 * to the local cache. Overwrites existing cached values. Returns success. */
	virtual bool ReadArrayInformation( const std::string& arrayPath );
	
	/*! \brief Read out the fiducial array from the cache. Throws an exception if not cached. */
	const FiducialArray& GetFiducialArray( const std::string& arrayPath );
	
	/*! \brief Read out the parent fiducial array from the cache. Throws an exception if not cached. */
	const FiducialArray& GetParentFiducialArray( const std::string& fidName );
	
};
	
} // end namespace fiducial_array
