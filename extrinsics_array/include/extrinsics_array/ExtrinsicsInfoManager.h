#pragma once

#include "extrinsics_array/ExtrinsicsArray.h"
#include "extrinsics_array/ExtrinsicsArrayInfo.h"
#include "lookup/LookupInterface.h"

#include <ros/ros.h>
#include <unordered_map>
#include <unordered_set>

namespace extrinsics_array
{
	
/*! \brief Stores, retrieves, and caches extrinsics information from the ROS
 * parameter server. */
// TODO Provide services for saving extrinsics to disk?
class ExtrinsicsInfoManager
{
public:
	
	ExtrinsicsInfoManager( lookup::LookupInterface& interface );

	/*! \brief Load information for the array that contains the specified member using
	 * the lookup table. Overwrites existing cached values. If memberName has been
	 * queried before and failed, this method skips querying the parameter server unless
	 * forceRead is set. Returns success. */
	bool ReadMemberInformation( const std::string& memberName, bool forceRead = false );
	
	/*! \brief Returns whether the member is cached. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Return the array info from the cache. Throws an exception if not cached. */
	const argus_utils::PoseSE3& GetExtrinsics( const std::string& memberName ) const;
	
	const std::string& GetReferenceFrame( const std::string& memberName ) const;
	
protected:
	
	ros::NodeHandle nodeHandle;
	lookup::LookupInterface& lookupInterface;
	
	/*! \brief Records array queries that have failed before. */
	std::unordered_set< std::string > failedQueries;
	
	struct MemberRegistration
	{
		argus_utils::PoseSE3 extrinsics;
		std::string frameID;
	};
	
	std::unordered_map< std::string, MemberRegistration > memberRegistry;
	
};
	
} // end namespace extrinsics_array
