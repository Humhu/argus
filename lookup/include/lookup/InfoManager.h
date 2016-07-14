#pragma once

#include <ros/duration.h>
#include "lookup/LookupInterface.h"
#include <unordered_map>
#include <unordered_set>

namespace argus
{

/*! \brief Basic lookup interface for information manager classes that use the
 * lookup system to retrieve/store object information. */
template <typename InfoStruct>
class InfoManager
{
public:

	typedef std::shared_ptr<InfoManager> Ptr;
	typedef InfoStruct InfoType;
	
	InfoManager( LookupInterface& interface );
	virtual ~InfoManager();
	
	/*! \brief Checks to see if information is cached. If it is not, attempts
	 * to read the information. Returns whether the information is now cached. */
	bool CheckMemberInfo( const std::string& memberName, bool forceLookup = false,
	                      const ros::Duration& timeout = ros::Duration( 0 ) );
	
	/*! \brief Attempt to read information for the specified member name. Fast-returns
	 * if previous lookup failures exist and the forceLookup flag is not set. 
	 * Returns success. */
	bool ReadMemberInfo( const std::string& memberName,
	                     bool forceLookup = false,
	                     const ros::Duration& timeout = ros::Duration( 0 ) );
	
	/*! \brief Attempt to write cached information for the specified member name. 
	 * Fast-returns if previous lookup failures exist and the forceLookup flag 
	 * is not set. Returns success. */
	bool WriteMemberInfo( const std::string& memberName,
	                      bool forceLookup = false,
	                      const ros::Duration& timeout = ros::Duration( 0 ) );
	
	bool WriteMemberInfo( const std::string& memberName, const InfoStruct& info,
	                      bool forceLookup = false,
	                      const ros::Duration& timeout = ros::Duration ( 0 ) );

	/*! \brief Returns whether the member information is cached. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Retrieve the member information registration. */
	InfoStruct& GetInfo( const std::string& memberName );
	const InfoStruct& GetInfo( const std::string& memberName ) const;
	const std::string& GetNamespace( const std::string& memberName ) const;

protected:
	
	/*! \brief Returns the prefix namespace for the specified member. */
	bool GetNamespace( const std::string& memberName, std::string& ns, 
	                   bool forceLookup, const ros::Duration& timeout );

	/*! \brief Reads fields from memberNamespace into the info struct.
	 * memberNamespace always ends in a trailing backslash (/) */
	virtual bool ParseMemberInfo( const std::string& memberNamespace,
	                              InfoStruct& info ) = 0;

	/*! \brief Writes fields from the info struct to the member namespace. 
	 * memberNamespace always ends in a trailing backslash (/) */
	virtual void PopulateMemberInfo( const InfoStruct& info,
	                                 const std::string& memberNamespace ) = 0;
	
	/*! \brief Record a failed lookup for the specified member. */
	void RecordFailure( const std::string& memberName );

	/*! \brief Purge failures for the specified member. */
	void ClearFailures( const std::string& memberName );
	
	/*! \brief Returns whether the specified member has failed lookups on record. */
	bool HasFailures( const std::string& memberName ) const;
	
private:
	
	LookupInterface& lookupInterface;
	
	struct MemberRegistration
	{
		std::string nameSpace;
		InfoStruct info;
	};

	/*! \brief Records array queries that have failed before. */
	std::unordered_set <std::string> failedQueries;
	std::unordered_map <std::string, MemberRegistration> registry;
};
	
}

#include "lookup/InfoManager.hpp"
