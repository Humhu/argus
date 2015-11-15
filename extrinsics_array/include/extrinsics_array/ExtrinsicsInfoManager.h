#pragma once

#include "extrinsics_array/ExtrinsicsArray.h"
#include "extrinsics_array/ExtrinsicsArrayInfo.h"
#include "lookup/LookupInterface.h"

#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>

namespace extrinsics_array
{
	
/*! \brief Stores, retrieves, and caches array information from the ROS
 * parameter server. Also manages the array member lookup table on the parameter
 * server. 
 *
 * Arrays are referred to by their parameter path, always ending in a slash (/). 
 * This can be absolute or relative, so long as a parameter server query to the 
 * path returns a struct that can be parsed by the calibration parsing tools.
 *
 * The lookup service stores array paths for each member, allowing extrinsics 
 * lookup from just the member name. Loading an extrinsic calibration file through
 * the manager automatically sets up the appropriate lookup parameters. */
// TODO Deal with 'none' parents orphans
class ExtrinsicsInfoManager
{
public:
	
	ExtrinsicsInfoManager( ros::NodeHandle& nh );
	
	virtual ~ExtrinsicsInfoManager();
	
	/*! \brief Specify the path to the lookup namespace. This can be relative
	 * or absolute. Automatically appends a '/' to the end if not present. */
	void SetLookupNamespace( const std::string& loc );
	
	/*! \brief Load information for an array from the parameter server to the local cache. 
	 * Overwrites existing cached values. If arrayPath has been queried before and 
	 * failed, this method skips querying the parameter server unless forceRead is set.
	 * Returns success. */
	virtual bool ReadArrayInformation( const std::string& arrayPath, bool forceRead = false );

	/*! \brief Load information for the array that contains the specified member using
	 * the lookup table. Overwrites existing cached values. If memberName has been
	 * queried before and failed, this method skips querying the parameter server unless
	 * forceRead is set. Returns success. */
	bool ReadMemberInformation( const std::string& memberName, bool forceRead = false );
	
	/*! \brief Returns whether the array at the path is cached. */
	bool HasArray( const std::string& arrayPath ) const;
	
	/*! \brief Returns whether the member is cached. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Return the array info from the cache. Throws an exception if not cached. */
	const ExtrinsicsArray& GetArray( const std::string& arrayPath );
	
	/*! \brief Return the parent array info from the cache. Throws an exception if not cached. */
	const ExtrinsicsArray& GetParentArray( const std::string& memberName );
	
	/*! \brief Return the parent array path from the cache. Throws an exception if not cached. */
	const std::string& GetParentPath( const std::string& memberName );
	
protected:
	
	ros::NodeHandle nodeHandle;
	
	lookup::LookupInterface lookupInterface;
	
	/*! \brief Records array queries that have failed before. */
	std::unordered_set< std::string > failedArrayQueries;
	std::unordered_set< std::string > failedMemberQueries;
	
	/*! \brief Map from array names to info. */
	std::unordered_map< std::string, ExtrinsicsArray::Ptr > arrayMap;
	
	/*! \brief Map from member names to arrays. */
	std::unordered_map< std::string, std::string > memberMap;
	
	static std::string SanitizeArrayPath( const std::string& arrayPath );
	
};
	
} // end namespace extrinsics_array
