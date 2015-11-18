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
	 * forceLookup is set. Returns success. */
	bool ReadMemberInformation( const std::string& memberName, bool forceLookup = false );
	bool WriteMemberInformation( const std::string& memberName, bool forceLookup = false );
	
	/*! \brief Returns whether the member is cached. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Return the extrinsics and parent reference frame from the cache. 
	 * Throws an exception if not cached. */
	const argus_utils::PoseSE3& GetExtrinsics( const std::string& memberName ) const;
	const std::string& GetReferenceFrame( const std::string& memberName ) const;
	
	/*! \brief Set the cached extrinsics and parent reference frame. */
	void SetExtrinsics( const std::string& memberName, const argus_utils::PoseSE3& ext );
	void SetReferenceFrame( const std::string& memberName, const std::string& refFrame );
	
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
	
	bool GetNamespace( const std::string& memberName, std::string& ns, bool forceLookup );
	static std::string GenerateExtrinsicsKey( const std::string& ns );
	static std::string GenerateRefFrameKey( const std::string& ns );
	
};
	
} // end namespace extrinsics_array
