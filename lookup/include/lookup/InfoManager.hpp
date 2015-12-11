#include "lookup/InfoManager.h"

namespace lookup
{

template <typename InfoStruct>
InfoManager<InfoStruct>::InfoManager( lookup::LookupInterface& interface )
: lookupInterface( interface )
{}

template <typename InfoStruct>
InfoManager<InfoStruct>::~InfoManager()
{}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::HasMember( const std::string& memberName ) const
{
	return registry.count( memberName ) > 0;
}

template <typename InfoStruct>
InfoStruct& InfoManager<InfoStruct>::GetInfo( const std::string& memberName )
{
	return registry.at( memberName );
}

template <typename InfoStruct>
const InfoStruct& InfoManager<InfoStruct>::GetInfo( const std::string& memberName ) const
{
	return registry.at( memberName );
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::GetNamespace( const std::string& memberName,
                                            std::string& ns,
                                            bool forceLookup )
{
	// Fast-fail using cache
	if( !forceLookup && HasFailures( memberName ) > 0 ) 
	{ 
		ROS_INFO_STREAM( "Previous failures on record for " << memberName
			<< " and forceLookup flag not set." );
		return false; 
	}
	
	if( !lookupInterface.ReadNamespace( memberName, ns ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << memberName );
		RecordFailure( memberName );
		return false;
	}
	return true;
}

template <typename InfoStruct>
void InfoManager<InfoStruct>::RecordFailure( const std::string& memberName )
{
	failedQueries.insert( memberName );
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::HasFailures( const std::string& memberName ) const
{
	return failedQueries.count( memberName ) > 0;
}

template <typename InfoStruct>
void InfoManager<InfoStruct>::RegisterMember( const std::string& memberName,
                                              const InfoStruct& info )
{
	registry[ memberName ] = info;
}
	
}
