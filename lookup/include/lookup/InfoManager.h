#pragma once

#include "lookup/LookupInterface.h"
#include <unordered_map>
#include <unordered_set>

namespace lookup
{

/*! \brief Basic lookup interface for information manager classes that use the
 * lookup system to retrieve/store object information. */
template <typename InfoStruct>
class InfoManager
{
public:

	typedef std::shared_ptr<InfoManager> Ptr;
	typedef InfoStruct InfoType;
	
	InfoManager( lookup::LookupInterface& interface );
	virtual ~InfoManager();
	
	/*! \brief Attempt to read information for the specified member name. Fast-returns
	 * if previous lookup failures exist and the forceLookup flag is not set. 
	 * Returns success. */
	virtual bool ReadMemberInfo( const std::string& memberName,
	                             bool forceLookup = false ) = 0;
	
	/*! \brief Attempt to write cached information for the specified member name. 
	 * Fast-returns if previous lookup failures exist and the forceLookup flag 
	 * is not set. Returns success. */
	virtual bool WriteMemberInfo( const std::string& memberName,
	                              bool forceLookup = false ) = 0;
	
	/*! \brief Returns whether the member information is cached. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Retrieve the member information registration. */
	InfoStruct& GetInfo( const std::string& memberName );
	const InfoStruct& GetInfo( const std::string& memberName ) const;
	
protected:
	
	/*! \brief Returns the prefix namespace for the specified member. */
	bool GetNamespace( const std::string& memberName, std::string& ns, bool forceLookup );
	
	/*! \brief Record a failed lookup for the specified member. */
	void RecordFailure( const std::string& memberName );
	
	/*! \brief Returns whether the specified member has failed lookups on record. */
	bool HasFailures( const std::string& memberName ) const;
	
	/*! \brief Assigns the info to the specified member name. Overwrites
	 * old info if it exists. */
	void RegisterMember( const std::string& memberName, const InfoStruct& info );
	
	
private:
	
	lookup::LookupInterface& lookupInterface;
	
	/*! \brief Records array queries that have failed before. */
	std::unordered_set <std::string> failedQueries;
	
	std::unordered_map <std::string, InfoStruct> registry;
};
	
}

#include "lookup/InfoManager.hpp"
