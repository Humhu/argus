#include "lookup/InfoManager.h"

#define RETRY_WAIT_TIME (0.1)

namespace argus
{

template <typename InfoStruct>
InfoManager<InfoStruct>::InfoManager( LookupInterface& interface )
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
	if( !forceLookup && HasMember( memberName ) ) 
	{ 
		return true; 
	}
	
	//ros::Time start = ros::Time::now();
	int numRetries = std::ceil( timeout.toSec() / RETRY_WAIT_TIME );
	if( ReadMemberInfo( memberName, forceLookup ) ) 
	{ 
		ClearFailures( memberName );
		return true; 
	}
	else if( timeout.toSec() == 0.0 ) { return false; }

	while( numRetries > 0 )
	{
		if( ReadMemberInfo( memberName, forceLookup ) ) 
		{ 
			ClearFailures( memberName );
			return true; 
		}
		ROS_INFO_STREAM( "Lookup failed. Retrying..." );
		ros::Duration( RETRY_WAIT_TIME ).sleep(); // TODO Make a parameter
		// if( ros::Time::now() > start + timeout ) { return false; }
		numRetries--;
	}
	return false;
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::ReadMemberInfo( const std::string& memberName,
                                              bool forceLookup,
                                              const ros::Duration& timeout )
{
	MemberRegistration reg;
	// Get namespace records failures, so we don't have to here
	if( !GetNamespace( memberName, reg.nameSpace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}

	InfoStruct info;
	if( !ParseMemberInfo( reg.nameSpace, reg.info ) ) 
	{ 
		RecordFailure( memberName );
		return false; 
	}

	registry[memberName] = reg;
	return true;
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::WriteMemberInfo( const std::string& memberName,
                                               bool forceLookup,
                                               const ros::Duration& timeout )
{
	// Can't write info if we don't have it cached!
	if( !HasMember( memberName ) ) { return false; }
	
	std::string memberNamespace;
	// Get namespace records failures, so we don't have to here
	if( !GetNamespace( memberName, memberNamespace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}

	PopulateMemberInfo( GetInfo( memberName ), memberNamespace );
	return true;
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::WriteMemberInfo( const std::string& memberName, 
                                               const InfoStruct& info,
                                               bool forceLookup ,
                                               const ros::Duration& timeout )
{
	MemberRegistration reg;
	reg.info = info;
	// Get namespace records failures, so we don't have to here
	if( !GetNamespace( memberName, reg.nameSpace, forceLookup, timeout ) ) 
	{ 
		return false; 
	}

	PopulateMemberInfo( info, reg.nameSpace );
	registry[memberName] = reg;
	return true;
}

template <typename InfoStruct>
bool InfoManager<InfoStruct>::HasMember( const std::string& memberName ) const
{
	return registry.count( memberName ) > 0;
}

template <typename InfoStruct>
InfoStruct& 
InfoManager<InfoStruct>::GetInfo( const std::string& memberName )
{
	return registry.at( memberName ).info;
}

template <typename InfoStruct>
const InfoStruct& 
InfoManager<InfoStruct>::GetInfo( const std::string& memberName ) const
{
	return registry.at( memberName ).info;
}

template <typename InfoStruct>
const std::string& 
InfoManager<InfoStruct>::GetNamespace( const std::string& memberName ) const
{
	return registry.at( memberName ).nameSpace;
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
	
	if( registry.count( memberName ) > 0 )
	{
		ns = registry[ memberName ].nameSpace;
		return true;
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

}
