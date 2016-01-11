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
bool InfoManager<InfoStruct>::CheckMemberInfo( const std::string& memberName, 
                                               bool forceLookup,
                                               const ros::Duration& timeout )
{
	if( HasMember( memberName ) ) 
	{ 
		ClearFailures( memberName );
		return true; 
	}
	
	ros::Time start = ros::Time::now();
	if( ReadMemberInfo( memberName, forceLookup ) ) 
	{ 
		ClearFailures( memberName );
		return true; 
	}
	else if( timeout.toSec() == 0.0 ) { return false; }

	while( true )
	{
		if( ReadMemberInfo( memberName, forceLookup ) ) 
		{ 
			ClearFailures( memberName );
			return true; 
		}
		ROS_INFO_STREAM( "Lookup failed. Retrying..." );
		ros::Duration( 0.5 ).sleep();
		if( ros::Time::now() > start + timeout ) { return false; }
	}
}

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
                                            bool forceLookup,
                                            const ros::Duration& timeout )
{
	// Fast-fail using cache
	if( !forceLookup && HasFailures( memberName ) > 0 ) 
	{ 
		ROS_INFO_STREAM( "Previous failures on record for " << memberName
			<< " and forceLookup flag not set." );
		return false; 
	}
	
	if( !lookupInterface.ReadNamespace( memberName, ns, timeout ) )
	{
		ROS_WARN_STREAM( "Could not find namespace for: " << memberName );
		RecordFailure( memberName );
		return false;
	}

	ClearFailures( memberName );
	return true;
}

template <typename InfoStruct>
void InfoManager<InfoStruct>::RecordFailure( const std::string& memberName )
{
	failedQueries.insert( memberName );
}

template <typename InfoStruct>
void InfoManager<InfoStruct>::ClearFailures( const std::string& memberName )
{
	failedQueries.erase( memberName );
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
